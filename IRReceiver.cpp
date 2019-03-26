#include "IRReceiver.h"

IRReceiver::IRReceiver()
{
    irpin = 7;
    pinMode(irpin,INPUT);
    ir_code=0x00;// 清零  
    adrL_code=0x00;// 清零  
    adrH_code=0x00;// 清零   

}

IRReceiver::IRReceiver(byte pin)
{
    irpin = pin;
    pinMode(irpin,INPUT);
    ir_code=0x00;// 清零  
    adrL_code=0x00;// 清零  
    adrH_code=0x00;// 清零   
}


int IRReceiver::readIRCode(IRCode &code)
{
    code.addr = 0;
    code.code = 0;

    int in = digitalRead(irpin);
    { 
          digitalWrite( 13, !in);

        if( in == 1 )
            return -1;
        else
        {
            prevMillis = millis();
            while( !digitalRead(irpin));
            curMillis = millis();
            pulse_width_l = curMillis - prevMillis;
            prevMillis = curMillis;
            while( !digitalRead(irpin));
            curMillis = millis();
            pulse_width_h = curMillis - prevMillis;
            prevMillis = curMillis;
            if( pulse_width_l > 8000 && pulse_width_h < 2500 ) //start pulse
            {
                readCode();
            }
            else
                return -1;
        }
    }

    code.addr = ir_code;
    code.code = adrL_code;
    Serial.print("ir rec:");
    Serial.print(ir_code);
    Serial.print(", ");
    Serial.println(adrL_code);
    return 0;
}


void IRReceiver::readCode()
{

    int i;  
    int j;  
    //解析遥控器编码中的用户编码值    
    for(i = 0 ; i < 16; i++)  
    {    
        if(logicValue() == 1) //是1        
            ir_code |= (1<<i);//保存键值  
    }  
    //解析遥控器编码中的命令码  
    for(i = 0 ; i < 8; i++)  
    {    
        if(logicValue() == 1) //是1      
        adrL_code |= (1<<i);//保存键值  
    }  
    //解析遥控器编码中的命令码反码   
    for(j = 0 ; j < 8; j++)  
    {    
        if(logicValue() == 1) //是1        
            adrH_code |= (1<<j);//保存键值  
    }
}

int IRReceiver::logicValue()
{
 
      while( !digitalRead(irpin));
        curMillis = millis();
            pulse_width_l = curMillis - prevMillis;
            prevMillis = curMillis;

            while( !digitalRead(irpin));
            curMillis = millis();
            pulse_width_h = curMillis - prevMillis;
            prevMillis = curMillis;

            if( pulse_width_l < 600 && pulse_width_h < 600 ) //start pulse
            {
                return 0;
            }
            else if( pulse_width_l < 600 && pulse_width_h > 1000 )
                return 1;
            else
                return -1; 
}