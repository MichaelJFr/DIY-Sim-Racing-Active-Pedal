#pragma once

#include "DiyActivePedal_types.h"
#include "ForceCurve.h"

int32_t MoveByLinearStrategy(float filteredLoadReadingKg, const DAP_calculationVariables_st& calc_st) {
  float spingStiffnessInv_lcl = calc_st.springStiffnesssInv;
  // caclulate pedal position
  return spingStiffnessInv_lcl * (filteredLoadReadingKg - calc_st.Force_Min) + calc_st.stepperPosMin ;        //Calculates new position using linear function
}

int32_t MoveByInterpolatedStrategy(float filteredLoadReadingKg, float stepperPosFraction, const ForceCurve_Interpolated* forceCurve, const DAP_calculationVariables_st& calc_st) {
  float spingStiffnessInv_lcl = calc_st.springStiffnesssInv;
  float springStiffnessInterp = forceCurve->stiffnessAtPosition(stepperPosFraction);
  if (springStiffnessInterp > 0) {
    spingStiffnessInv_lcl = (1.0f / springStiffnessInterp);
  }

  // caclulate pedal position
  float pedalForceInterp = forceCurve->forceAtPosition(stepperPosFraction);
  float stepperPosInterp = forceCurve->stepperPos(stepperPosFraction);
  return spingStiffnessInv_lcl * (filteredLoadReadingKg - pedalForceInterp) + stepperPosInterp;
}


int32_t MoveByForceTargetingStrategy(float loadCellReadingKg, StepperWithLimits* stepper, ForceCurve* forceCurve) {
  float stepperPosFraction = stepper->getCurrentPositionFraction();
  float loadCellTargetKg = forceCurve->forceAtPosition(stepperPosFraction);
  float loadCellErrorKg = loadCellReadingKg - loadCellTargetKg;

  // how many mm movement to order if 1kg of error force is detected
  // this can be tuned for responsiveness vs oscillation
  static const float MOVE_MM_FOR_1KG = 3.0;
  static const float MOVE_STEPS_FOR_1KG = (MOVE_MM_FOR_1KG / TRAVEL_PER_ROTATION_IN_MM) * STEPS_PER_MOTOR_REVOLUTION;

  // square the error to smooth minor variations
  float loadCellErrorMultipler = loadCellErrorKg;
  if (loadCellErrorKg < 0.0) {
    loadCellErrorMultipler = sq(max(-1.0f, loadCellErrorKg) / min(1.0f, loadCellTargetKg)) * -1.0;
  } else if (loadCellErrorKg < 1.0) {
    loadCellErrorMultipler = sq(loadCellErrorKg);
  }

  int32_t posStepperChange = loadCellErrorMultipler * MOVE_STEPS_FOR_1KG;
  int32_t posStepper = stepper->getCurrentPositionSteps();
  int32_t posStepperNew = posStepper + posStepperChange;

  bool overshoot = false;
  do {   // check for overshoot
    float stepperPosFractionNew = stepperPosFraction + (posStepperChange / stepper->getTravelSteps());
    float loadCellTargetKgAtPosNew = forceCurve->forceAtPosition(stepperPosFractionNew);

    overshoot = 
      (loadCellTargetKg < loadCellReadingKg && loadCellReadingKg < loadCellTargetKgAtPosNew) ||
      (loadCellTargetKg > loadCellReadingKg && loadCellReadingKg > loadCellTargetKgAtPosNew);

    if (overshoot) {
      int32_t corrected = (posStepper + posStepperNew) / 2;
      if (corrected == posStepperNew) {
        overshoot = false;
      } else {
        posStepperNew = corrected;   // and check again
      }
    }
  } while (overshoot);

  return posStepperNew;
}
