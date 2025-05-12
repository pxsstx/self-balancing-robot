#include "IRController.h"

IRController::IRController(int recvPin) : irrecv(recvPin), lastCode(0), lastCommandTime(0) {}

void IRController::begin() {
    irrecv.enableIRIn();
}

bool IRController::available() {
    return irrecv.decode(&results);
}

IRCommand IRController::getCommand() {
    unsigned long code;

    if (results.value == 0xFFFFFFFF) {
        code = lastCode;
    } else {
        code = results.value;
        lastCode = code;
    }

    switch (code) {
        case 0xFF18E7: return IRCommand::Forward;
        case 0xFF4AB5: return IRCommand::Backward;
        case 0xFF10EF: return IRCommand::Left;
        case 0xFF5AA5: return IRCommand::Right;
        default:       return IRCommand::None;
    }
}

void IRController::resume() {
    irrecv.resume();
}

void IRController::updateLastCommandTime() {
    lastCommandTime = millis();
}

unsigned long IRController::getLastCommandTime(){
    return lastCommandTime;
}
