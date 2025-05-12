#ifndef IRController_h
#define IRController_h

#include <IRremote.h>  // library สำหรับ IR

enum class IRCommand {
    None,
    Forward,
    Backward,
    Left,
    Right
};

class IRController {
public:
    IRController(int recvPin);
    void begin();

    bool available();                          // ตรวจว่ามีสัญญาณเข้ามาไหม
    IRCommand getCommand();                    // แปลงรหัสเป็นคำสั่ง
    void resume();                             // Resume IR receiver
    void updateLastCommandTime();              // อัปเดตเวลาคำสั่งล่าสุด
    unsigned long getLastCommandTime();        // ดึงเวลาล่าสุด

private:
    IRrecv irrecv; 
    decode_results results;
    unsigned long lastCode; 
    unsigned long lastCommandTime;
};


#endif
