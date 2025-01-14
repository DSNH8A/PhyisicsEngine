
#include "core.h"


const Vector3 Vector3::GRAVITY = Vector3(0, -9.81, 0);
const Vector3 Vector3::HIGH_GRAVITY = Vector3(0, -19.62, 0);
const Vector3 Vector3::UP = Vector3(0, 1, 0);
const Vector3 Vector3::RIGHT = Vector3(1, 0, 0);
const Vector3 Vector3::OUT_OF_SCREEN = Vector3(0, 0, 1);
const Vector3 Vector3::X = Vector3(0, 1, 0);
const Vector3 Vector3::Y = Vector3(1, 0, 0);
const Vector3 Vector3::Z = Vector3(0, 0, 1);

/*
 * Definition of the sleep epsilon extern.
 */
real sleepEpsilon = ((real)0.3);

/*
 * Functions to change sleepEpsilon.
 */
void setSleepEpsilon(real value)
{
    sleepEpsilon = value;
}

real getSleepEpsilon()
{
    return sleepEpsilon;
}