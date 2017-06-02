#pragma once

#include <avr/io.h>
#include <avr/interrupt.h>
#include "Arduino.h"

// Positions are generally in 'rotations' except where explicitly marked 'step'

class GrubStep {
public:
	struct Settings {
		int stepPin = 8;
		int directionPin = 9;
		int enabledPin = 10;

		bool enabledPolarity = LOW;

		float stepsPerRotation = 360.0f / 1.8f;

		float maximumVelocity = 1.0f; // rotations per second
		float maximumAcceleration = 1.0f; // rotations per (second)^2
	};

	GrubStep();
	~GrubStep() = delete;

	void begin(const Settings & settings);
	void update();

	float getPosition() const;

	void driveTo(float position);
	void stop();

	int getStepPosition() const;
	void tareStepPosition(int stepPosition = 0);

	float getVelocity() const;
	float getVelocitySteps() const;

	static GrubStep * & getFirst();
	GrubStep * & getNext();
	void tick();
protected:
	Settings settings;

	bool tickPolarity = true;
	unsigned long ticksSinceLastStep = 0;
	unsigned long ticksPerStep = 10000;

	volatile bool movementEnabled = false;
	volatile bool stepDirection = true;
	volatile long long currentPositionSteps = 0;
	volatile long long targetPositionSteps = 0;

	float currentVelocity = 0.0f;
	float currentAcceleration = 0.0f;

	float targetPosition = 0.0f;

	unsigned long long lastFrameTime = 0;
private:
	GrubStep * next = nullptr;
	void initialiseTimer() const;
};