#pragma once

#include "DiyActivePedal_types.h"


// interface - all force curves must implement forceAtPosition
class ForceCurve {
public:
  virtual float forceAtPosition(float fractionalPos) const = 0;
};


class ForceCurve_ConstantForce : public ForceCurve {
private:
  float _constantForce;
public:
  ForceCurve_ConstantForce(float constantForce) : _constantForce(constantForce) {}
  virtual float forceAtPosition(float fractionalPos) const override { return _constantForce; }
};

class ForceCurve_LinearSpring : public ForceCurve {
private:
  float _forceMin, _forceMax, _forceRange;
public:
  ForceCurve_LinearSpring(float forceMin, float forceMax)
    : _forceMin(forceMin), _forceMax(forceMax)
    , _forceRange(forceMax - forceMin)
  {}
  virtual float forceAtPosition(float fractionalPos) const override {
    if (fractionalPos <= 0) return _forceMin;
    if (fractionalPos >= 1) return _forceMax;
    return _forceMin + (fractionalPos * _forceRange);
  }
};



static const int INTERPOLATION_NUMBER_OF_SOURCE_VALUES = 6;
static const int INTERPOLATION_NUMBER_OF_TARGET_VALUES = 30;

class ForceCurve_Interpolated : public ForceCurve {
private:
  float _stepperPos[INTERPOLATION_NUMBER_OF_TARGET_VALUES];
  float _targetValues[INTERPOLATION_NUMBER_OF_TARGET_VALUES];
  float _springStiffness[INTERPOLATION_NUMBER_OF_TARGET_VALUES];

public:
  ForceCurve_Interpolated(DAP_config_st& config_st, DAP_calculationVariables_st& calc_st);

private:
  int fractionalPosToIndex(float fractionalPos) const;

public:
  float stepperPos(float fractionalPos) const           { return _stepperPos[fractionalPosToIndex(fractionalPos)]; }
  float stiffnessAtPosition(float fractionalPos) const  { return _springStiffness[fractionalPosToIndex(fractionalPos)]; }
  
  virtual float forceAtPosition(float fractionalPos) const override {
    return _targetValues[fractionalPosToIndex(fractionalPos)];
  }
};
