#ifndef ENDSTOP_H_
#define ENDSTOP_H_

// Class that functions for interacting with end-stop.
class endstop {
public:

    endstop(uint8_t pin, bool default_state) {
        _pin = pin; //pin, set the arduino pin
        _default_state = default_state; // Set the default state of end-stop HIGH or LOW
    }

    // Initialize the Input pin for end-stop
    void init() {
        pinMode(_pin, INPUT_PULLUP);
    }

    // Get the state of end-stop
    bool get_state() {
        if (digitalRead(_pin) == _default_state)
            return true; // Return True if end-stop is triggered
        else
            return false;
    }

private:
    uint8_t _pin;
    bool _default_state;
};

#endif /* ENDSTOP_H_ */
