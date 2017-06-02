#include "GrubStep.h"
#include "GrubSep_digitalWriteFast.h"

template<typename Type>
Type sign(Type value) {
	if(value > 0) {
		return 1;
	} else if (value == 0) {
		return 0;
	} else {
		return -1;
	}
}

#define MAX(A, B) (A > B ? A : B)
#define MIN(A, B) (A < B ? A : B)

//---------
GrubStep::GrubStep() {
	//register this instance at the end of the chain
	GrubStep * next = GrubStep::getFirst();
	if(!next) {
		//we're the first
		GrubStep::getFirst() = this;
	} else {
		//search for last
		GrubStep * last;
		do {
			last = next;
			next = next->getNext();
		} while(next);
		last->getNext() = this;
	}
}

//---------
void GrubStep::begin(const Settings & settings) {
	this->settings = settings;

	pinMode(this->settings.stepPin, OUTPUT);
	pinMode(this->settings.directionPin, OUTPUT);
	pinMode(this->settings.enabledPin, OUTPUT);

	digitalWrite(this->settings.directionPin, LOW);
	digitalWrite(this->settings.enabledPin, this->settings.enabledPolarity);

	this->initialiseTimer();
}

//---------
void GrubStep::update() {
	auto currentPosition = this->getPosition();
	unsigned long long time = micros();

	unsigned long long timeStep = time - this->lastFrameTime;
	auto timeStepSeconds = float(timeStep) / float(1e6);

	auto deltaPosition = this->targetPosition - currentPosition;
	auto direction = sign(deltaPosition);

	//print the time
	Serial.println(float(time / 1000) / 1000);

	if(targetPosition == currentPosition) {
		Serial.println("Stop");
		//stopped at destination, just be still
	}
	else if (this->currentVelocity == 0.0f || (direction != sign(currentVelocity))) {
		Serial.println("Start up motor / Switch direction");
		//needs to start moving in right direction
		this->currentVelocity += direction * (this->settings.maximumAcceleration * timeStepSeconds);
	} else {
		//already moving in right direction
		float timeUntilStop = (abs(this->currentVelocity) / this->settings.maximumAcceleration); // + calculation time
		float timeUntilArrive = deltaPosition / this->currentVelocity;

		// Serial.print("Time until stop : ");
		// Serial.println(timeUntilStop);
		// Serial.print("Time until arrive : ");
		// Serial.println(timeUntilArrive);

		if(timeUntilArrive > timeUntilStop) {
			//accelrate towards destination
			Serial.println("Accelerate towards destination");
			this->currentVelocity += direction * (this->settings.maximumAcceleration * timeStepSeconds);
		} else {
			//now is time to stop
			if(abs(this->currentVelocity) > 0) {
				// Serial.println("Decellerate to stop");
				//decellerate
				this->currentVelocity -= direction * (this->settings.maximumAcceleration * timeStepSeconds);
			} else {
				// Serial.println("Final steps");
				//TODO : manually perform the final steps
			}
			
		}
	}

	//limit velocity
	if(abs(this->currentVelocity) > this->settings.maximumVelocity) {
		this->currentVelocity = sign(this->currentVelocity) * this->settings.maximumVelocity;
	}

	//calculate movement state
	{
		float stepsPerSecond = this->getVelocitySteps();
		// Serial.print("Steps per second : ");
		// Serial.println(stepsPerSecond);

		float stepInterval = 1.0f / stepsPerSecond;
		float ticksPerStep = GRUBSTEP_TICKS_PER_SECOND * stepInterval;
		// Serial.print("Ticks per step : ");
		// Serial.println(ticksPerStep);

		if(stepsPerSecond > 0) {
			this->ticksPerStep = 100;//HACK	floor(ticksPerStep);
		}
	}
	
	this->stepDirection = this->currentVelocity > 0.0f;
	digitalWrite(this->settings.directionPin, this->stepDirection);

	this->lastFrameTime = time;
}

//---------
float GrubStep::getPosition() const {
	return float(this->getStepPosition()) / this->settings.stepsPerRotation;
}

//---------
void GrubStep::driveTo(float position) {
	this->targetPosition = position;
	this->targetPositionSteps = position * this->settings.stepsPerRotation;
}

//---------
int GrubStep::getStepPosition() const {
	return this->currentPositionSteps ;
}

//---------
float GrubStep::getVelocity() const {
	return this->currentVelocity;
}

//---------
float GrubStep::getVelocitySteps() const {
	return this->currentVelocity * this->settings.stepsPerRotation;
}

//---------
void GrubStep::tareStepPosition(int stepPosition) {
	this->currentPositionSteps = stepPosition;
}

//---------
GrubStep * & GrubStep::getFirst() {
	static GrubStep * firstGrubStep = nullptr;
	return firstGrubStep;
}

//---------
GrubStep * & GrubStep::getNext() {
	return this->next;
}

//---------
void GrubStep::tick() {

	//ticks are alternatively used for:
	// * Sending pulses
	// * Clearing pulses

	if(this->tickPolarity) {

		this->ticksSinceLastStep++;

		if(this->currentPositionSteps != this->targetPositionSteps) {

			//if counter hits, do the step
			if(this->ticksSinceLastStep >= this->ticksPerStep) {
				digitalWrite(this->settings.stepPin, HIGH);

				//record the step
				if(this->stepDirection) {
					++this->currentPositionSteps;
				} else {
					--this->currentPositionSteps;
				}

				this->ticksSinceLastStep = 0;
			}	
			this->tickPolarity = false;
		}
	}
	else {
		digitalWrite(this->settings.stepPin, LOW);
		this->tickPolarity = true;
	}
}

//---------
void GrubStep::initialiseTimer() const {
	static bool timerInitialised = false;

	if(timerInitialised) {
		return;
	}

	{
		// initialize timer1 
		noInterrupts();           // disable all interrupts
	
		//clear timer registers
		TCCR1A = 0;
		TCCR1B = 0;
	
		//enable CTC mode (compare)
		TCCR1B |= (1 << WGM12);
	
		// enable timer1 compare interrupt
		TIMSK1 |= (1 << OCIE1A);  

		//TCCR1B |= (1 << CS11);    // 8 prescaler 
		TCCR1B |= (1 << CS11) | (1 << CS10);    // 64 prescaler 
		//TCCR1B |= (1 << CS12);    // 256 prescaler 
		//TCCR1B |= (1 << CS12) | (1 << CS10);    // 1024 prescaler 

		OCR1A = 2 - 1; // takes one cycle for operation (?)

		interrupts();             // enable all interrupts
	}

	timerInitialised = true;
}

//---------
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
	auto instance = GrubStep::getFirst();
	instance->tick();
	return; //HACK
	while(instance) {
		instance->tick();
		instance = instance->getNext();
	}
}