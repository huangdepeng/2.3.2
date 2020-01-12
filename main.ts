
/*
 hicbit_control package
*/
//% weight=10 icon="\uf2c5" color=#7CCD7C
namespace hicbit_control {

    export enum hicbit_key {
        //% block="up"
        up = 0x01,
        //% block="down"
        down = 0x02,
        //% block="left"
        left = 0x03,
        //% block="right"
        right = 0x04
    }
    
    /**
     * hicbit initialization, please execute at boot time
    */
    //% weight=100 blockGap=20 blockId=hicbit_Init block="Initialize hicbit"
    export function hicbit_Init() {

        led.enable(false);

        serial.redirect(
            SerialPin.P8,
            SerialPin.P12,
            BaudRate.BaudRate115200);

        basic.forever(() => {
            getHandleCmd();
        });
        serial.writeString(Display.NEW_LINE);
        basic.pause(500);
        Display.Clearscreen();
        basic.pause(500);
    }

    let handleCmd: string = "";
    
    /**
    * Get the handle command.
    */
    function getHandleCmd() {
        let charStr: string = serial.readString();
        handleCmd = handleCmd.concat(charStr);
        handleCmd = "";
    }

    /**
     * Pause for the specified time in seconds
     * @param s how long to pause for, eg: 1, 2, 5, 10, 20,
     */
    //% weight=90
    //% block="wait(s) %s"
    //% blockId=wait_s
    export function wait_s(s:number) {
        basic.pause(s*1000);
    }

    /**
     * Pause for the specified time in milliseconds
     * @param ms how long to pause for, eg: 100, 200, 500
     */
    //% weight=89
    //% block="wait(ms) %ms"
    //% blockId=wait_ms
    export function wait_ms(ms:number) {
        basic.pause(ms);
    }

    /**
    * Set the arrow keys
    */
    //% weight=99 blockId=hicbit_Arrowkeys block="Arrow keys are|key %key"
    export function hicbit_Arrowkeys(key: hicbit_key): boolean {
        let status = 0;
        let flag: boolean = false;
        switch (key) {
            case hicbit_key.up:
                pins.setPull(DigitalPin.P5, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P5);
                break;
            case hicbit_key.down:
                pins.setPull(DigitalPin.P6, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P6);
                break;
            case hicbit_key.left:
                pins.setPull(DigitalPin.P7, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P7);
                break;
            case hicbit_key.right:
                pins.setPull(DigitalPin.P9, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P9);
                break;
        }
        if (status == 1)
            flag = false;
        else
            flag = true;
        return flag;
    }

}


/*
 hicbit package
*/
//% weight=9 icon="\uf180" color=#5F9EA0
namespace hicbit {

    export let NEW_LINE = "\r\n";

    export enum hicbit_Port {
        //% block="port A"
        port1 = 0x01,
        //% block="port B"
        port2 = 0x02,
        //% block="port C"
        port3 = 0x03,
        //% block="Port D"
        port4 = 0x04
    }

    export enum hicbit_Coded_motor_Port {
        //% block="port A"
        port1 = 0x01,
        //% block="port B"
        port2 = 0x02,
    }

    export enum Coded_motor_speed {
        //% block="fast"
        fast = 0x01,
        //% block="Medium"
        Medium = 0x02,
        //% block="slow"
        slow = 0x03,
    }

    /**
    *	Set interface motor speed , range of -255~255, that can control turn.
    */
    //% weight=99 blockId=hicbit_setMotorSpeed0 block="Set |port %port| motor|speed %speed|"
    //% speed.min=-255 speed.max=255 
    export function hicbit_setMotorSpeed0(port: hicbit_Port,speed: number) {
        let Turn: number = 0;//电机1：正 电机2：正
        let ports: number = 0;
        let speed1: number = 0;
        let speed2: number = 0;
        let buf = pins.createBuffer(5);

        if (speed > 255 || speed < -255) {
            return;
        }

        if (port == 1 || port == 3)
            speed1 = speed;
        else if (port == 2 || port == 4)
            speed2 = speed;
        
        if (port == 1 || port == 2)
            ports = 0;      //第一组tb6612
        else if (port == 3 || port == 4)
            ports = 1;      //第二组tb6612
        
        if (speed1 < 0) {
            speed1 = speed1 * -1;
            if (speed2 > 0)
                Turn = 1;//电机1：反 电机2：正
            else {
                speed2 = speed2 * -1;
                Turn = 3;//电机1：反 电机2：反
            }
        }
        else if (speed2 < 0) { 
            speed2 = speed2 * -1;
            if (speed1 > 0)
                Turn = 2;//电机1：正 电机2：反
            else {
                speed1 = speed1 * -1;
                Turn = 3;//电机1：反 电机2：反
            }
        }

        buf[0] = 0x58;      //标志位
        buf[1] = Turn;
        buf[2] = speed1;
        buf[3] = speed2;
        buf[4] = ports;
        serial.writeBuffer(buf);
        serial.writeString(NEW_LINE);

        basic.pause(200);
    }

    /**
    *	Stop motor.
    */
    //% weight=98 blockId=hicbit_StopMotor block="Set |port %port| motor stop"
    export function hicbit_StopMotor(port: hicbit_Port) {
        let buf = pins.createBuffer(5);

        buf[0] = 0x58;      //标志位
        buf[1] = 4;         //电机停止标志位
        buf[2] = 0;
        buf[3] = 0;
        buf[4] = port;      //接口
        serial.writeBuffer(buf);
        serial.writeString(NEW_LINE);

        basic.pause(200);
    }
    
    /**
    *	Set interface motor speed , range of -255~255, that can control turn.
    */
    //% weight=97 blockGap=50 blockId=hicbit_setMotorSpeed block="Set |port %port| motor|speed %speed|and|time(s) %time|"
    //% speed.min=-255 speed.max=255 
    //% time.min=0 time.max=20 
    export function hicbit_setMotorSpeed(port: hicbit_Port,speed: number,time:number) {
        let Turn: number = 0;//电机1：正 电机2：正
        let ports: number = 0;
        let time2: number = 0;
        let speed1: number = 0;
        let speed2: number = 0;
        let buf = pins.createBuffer(5);
        let buf2 = pins.createBuffer(5);

        if (speed > 255 || speed < -255) {
            return;
        } 

        if (port == 1 || port == 3)
            speed1 = speed;
        else if (port == 2 || port == 4)
            speed2 = speed;
        
        if (port == 1 || port == 2)
            ports = 0;      //第一组tb6612
        else if (port == 3 || port == 4)
            ports = 1;      //第二组tb6612
        
        if (speed1 < 0) {
            speed1 = speed1 * -1;
            if (speed2 > 0)
                Turn = 1;//电机1：反 电机2：正
            else {
                speed2 = speed2 * -1;
                Turn = 3;//电机1：反 电机2：反
            }
        }
        else if (speed2 < 0) { 
            speed2 = speed2 * -1;
            if (speed1 > 0)
                Turn = 2;//电机1：正 电机2：反
            else {
                speed1 = speed1 * -1;
                Turn = 3;//电机1：反 电机2：反
            }
        }
        
        buf[0] = 0x58;      //标志位
        buf[1] = Turn;
        buf[2] = speed1;
        buf[3] = speed2;
        buf[4] = ports;
        serial.writeBuffer(buf);
        serial.writeString(NEW_LINE);

        time2 = time * 1000;
        basic.pause(time2);
        
        buf2[0] = 0x58;      //标志位
        buf2[1] = Turn;
        buf2[2] = 0;
        buf2[3] = 0;
        buf2[4] = ports;
        serial.writeBuffer(buf2);
        serial.writeString(NEW_LINE);

        basic.pause(200);
    }

    /**
    *	Set the speed of the number 1 motor and number 2 motor, range of -255~255, that can control turn.
    */
    //% weight=96 blockId=hicbit_setMotorSpeed11 block="Set motor1 |speed %speed1|and motor2|speed %speed2|"
    //% speed1.min=-255 speed1.max=255
    //% speed2.min=-255 speed2.max=255
    export function hicbit_setMotorSpeed11(speed1: number, speed2: number) {
        let Turn: number = 0;//电机1：正 电机2：正

        if (speed1 > 255 || speed1 < -255 || speed2 > 255 || speed2 < -255) {
            return;
        } 
        if (speed1 < 0) {
            speed1 = speed1 * -1;
            if (speed2 > 0)
                Turn = 1;//电机1：反 电机2：正
            else {
                speed2 = speed2 * -1;
                Turn = 3;//电机1：反 电机2：反
            }
        }
        else if (speed2 < 0) { 
            speed2 = speed2 * -1;
            if (speed1 > 0)
                Turn = 2;//电机1：正 电机2：反
            else {
                speed1 = speed1 * -1;
                Turn = 3;//电机1：反 电机2：反
            }
        }
        let buf = pins.createBuffer(5);
        buf[0] = 0x58;
        buf[1] = Turn;
        buf[2] = speed1;
        buf[3] = speed2;
        buf[4] = 0;
        serial.writeBuffer(buf);
        serial.writeString(NEW_LINE);

        basic.pause(200);
    }

    /**
    *	Set the number 1 motor and number 2 motor stop.
    */
    //% weight=95 blockId=hicbit_setMotorstop1 block="Set motor1 and motor2 stop"
    export function hicbit_setMotorstop1() {
        let buf = pins.createBuffer(5);
        buf[0] = 0x58;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0;
        buf[4] = 0;
        serial.writeBuffer(buf);
        serial.writeString(NEW_LINE);

        basic.pause(200);
    }

    /**
    *	Set the speed of the number 1 motor and number 2 motor, range of -255~255, that can control turn.
    */
    //% weight=94 blockGap=50 blockId=hicbit_setMotorSpeed1 block="Set motor1|speed %speed1|and motor2|speed %speed2|and|time(s) %time|"
    //% speed1.min=-255 speed1.max=255
    //% speed2.min=-255 speed2.max=255
    //% time.min=0 time.max=20 
    export function hicbit_setMotorSpeed1(speed1: number, speed2: number,time:number) {
        let Turn: number = 0;//电机1：正 电机2：正
        let time2: number = 0;

        if (speed1 > 255 || speed1 < -255 || speed2 > 255 || speed2 < -255) {
            return;
        } 
        if (speed1 < 0) {
            speed1 = speed1 * -1;
            if (speed2 > 0)
                Turn = 1;//电机1：反 电机2：正
            else {
                speed2 = speed2 * -1;
                Turn = 3;//电机1：反 电机2：反
            }
        }
        else if (speed2 < 0) { 
            speed2 = speed2 * -1;
            if (speed1 > 0)
                Turn = 2;//电机1：正 电机2：反
            else {
                speed1 = speed1 * -1;
                Turn = 3;//电机1：反 电机2：反
            }
        }
        let buf = pins.createBuffer(5);
        buf[0] = 0x58;
        buf[1] = Turn;
        buf[2] = speed1;
        buf[3] = speed2;
        buf[4] = 0;
        serial.writeBuffer(buf);
        serial.writeString(NEW_LINE);

        time2 = time * 1000;
        basic.pause(time2);
        
        let buf2 = pins.createBuffer(5);
        buf2[0] = 0x58;      //标志位
        buf2[1] = Turn;
        buf2[2] = 0;
        buf2[3] = 0;
        buf2[4] = 0;
        serial.writeBuffer(buf2);
        serial.writeString(NEW_LINE);

        basic.pause(200);
    }

    /**
    *	Set the speed of the number 3 motor and number 4 motor, range of -255~255, that can control turn.
    */
    //% weight=93 blockId=hicbit_setMotorSpeed22 block="Set motor3 |speed %speed1|and motor4|speed %speed2|"
    //% speed1.min=-255 speed1.max=255
    //% speed2.min=-255 speed2.max=255
    export function hicbit_setMotorSpeed22(speed1: number, speed2: number) {
        let Turn: number = 0;//电机1：正 电机2：正

        if (speed1 > 255 || speed1 < -255 || speed2 > 255 || speed2 < -255) {
            return;
        } 
        if (speed1 < 0) {
            speed1 = speed1 * -1;
            if (speed2 > 0)
                Turn = 1;//电机1：反 电机2：正
            else {
                speed2 = speed2 * -1;
                Turn = 3;//电机1：反 电机2：反
            }
        }
        else if (speed2 < 0) { 
            speed2 = speed2 * -1;
            if (speed1 > 0)
                Turn = 2;//电机1：正 电机2：反
            else {
                speed1 = speed1 * -1;
                Turn = 3;//电机1：反 电机2：反
            }
        }
        let buf = pins.createBuffer(5);
        buf[0] = 0x58;
        buf[1] = Turn;
        buf[2] = speed1;
        buf[3] = speed2;
        buf[4] = 1;
        serial.writeBuffer(buf);
        serial.writeString(NEW_LINE);

        basic.pause(200);
    }

    /**
    *	Set the number 3 motor and number 4 motor stop.
    */
    //% weight=92 blockId=hicbit_setMotorstop2 block="Set motor3 and motor4 stop"
    export function hicbit_setMotorstop2() {
        let buf = pins.createBuffer(5);
        buf[0] = 0x58;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0;
        buf[4] = 1;
        serial.writeBuffer(buf);
        serial.writeString(NEW_LINE);

        basic.pause(200);
    }

    /**
    *	Set the speed of the number 3 motor and number 4 motor, range of -255~255, that can control turn.
    */
    //% weight=91 blockGap=50 blockId=hicbit_setMotorSpeed2 block="Set motor3 speed|%speed1|and motor4|speed %speed2|and|time(s) %time|"
    //% speed1.min=-255 speed1.max=255
    //% speed2.min=-255 speed2.max=255
    //% time.min=0 time.max=20 
    export function hicbit_setMotorSpeed2(speed1: number, speed2: number,time:number) {
        let Turn: number = 0;//电机1：正 电机2：正
        let time2: number = 0;

        if (speed1 > 255 || speed1 < -255 || speed2 > 255 || speed2 < -255) {
            return;
        } 
        if (speed1 < 0) {
            speed1 = speed1 * -1;
            if (speed2 > 0)
                Turn = 1;//电机1：反 电机2：正
            else {
                speed2 = speed2 * -1;
                Turn = 3;//电机1：反 电机2：反
            }
        }
        else if (speed2 < 0) { 
            speed2 = speed2 * -1;
            if (speed1 > 0)
                Turn = 2;//电机1：正 电机2：反
            else {
                speed1 = speed1 * -1;
                Turn = 3;//电机1：反 电机2：反
            }
        }
        let buf = pins.createBuffer(5);
        buf[0] = 0x58;
        buf[1] = Turn;
        buf[2] = speed1;
        buf[3] = speed2;
        buf[4] = 1;
        serial.writeBuffer(buf);
        serial.writeString(NEW_LINE);

        time2 = time * 1000;
        basic.pause(time2);
        
        let buf2 = pins.createBuffer(5);
        buf2[0] = 0x58;      //标志位
        buf2[1] = Turn;
        buf2[2] = 0;
        buf2[3] = 0;
        buf2[4] = 1;
        serial.writeBuffer(buf2);
        serial.writeString(NEW_LINE);

        basic.pause(200);
    }

    /**
    *	Set Coded motor , angle of -360~360, that can control turn.
    */
    //% weight=90 blockGap=50 blockId=hicbit_setCodedmotor block="Set |port %port| motor|angle %angle|and |speed %speed|"
    //% angle.min=-360 angle.max=360
    export function hicbit_setCodedmotor(port: hicbit_Coded_motor_Port,angle: number,speed:Coded_motor_speed) {
        let multiple: number = 1;   //倍数
        let turn: number = 0;
        let buf = pins.createBuffer(7);

        if (angle < 0)
        {
            turn = 1;           //反转
            angle *= -1;
        }
        else
            turn = 0;           //正转
        
        if (angle < 10)
            multiple = 1;
        else {
            multiple = 10;
            angle /= 10;
        }
        
        buf[0] = 0x59;      //标志位
        buf[1] = multiple;  //倍数
        buf[2] = angle;     //角度
        buf[3] = turn;      //正反转
        buf[4] = port - 1;  //端口
        buf[5] = 0;         //圈数
        buf[6] = speed;
        serial.writeBuffer(buf);
        serial.writeString(NEW_LINE);

        basic.pause(200);
    }

}


/*
 Sensor package
*/
//% weight=8 icon="\uf2db" color=#8470FF
namespace Sensor {
    
    export enum hicbit_Port {
        //% block="port 1"
        port1 = 0x01,
        //% block="port 2"
        port2 = 0x02,
        //% block="port 3"
        port3 = 0x03,
        //% block="Port 4"
        port4 = 0x04
    }

    export enum enRocker {
        //% blockId="Nostate" block="无"
        Nostate = 0,
        //% blockId="Up" block="上"
        Up,
        //% blockId="Down" block="下"
        Down,
        //% blockId="Left" block="左"
        Left,
        //% blockId="Right" block="右"
        Right,
    }

    export enum Dht11Result {
        //% block="Celsius"
        Celsius,
        //% block="Fahrenheit"
        Fahrenheit,
        //% block="humidity"
        humidity
    }


    export enum buzzer {
        //% block="ring"
        ring = 0x01,
        //% block="Not_ringing"
        Not_ringing = 0x02,
    }

    /**
        * Buzzer
        
    //% weight=100 blockId=Buzzer block="Buzzer(P0):| %buzzer"
    export function Buzzer(buz: buzzer): void {
        switch (buz) {
            case Sensor.buzzer.ring:
                pins.digitalWritePin(DigitalPin.P0, 1);
                break;
            case Sensor.buzzer.Not_ringing:
                pins.digitalWritePin(DigitalPin.P0, 0);
                break;
        }
    }*/

    /**
     * Get the line follower sensor port ad value 巡线
     */
    //% weight=99 blockId=hicbit_lineSensorValue block="Get line follower sensor Value|port %port|value(0~255)"
    export function hicbit_lineSensorValue(port: hicbit_Port): number {
        let ADCPin: AnalogPin;
        switch (port) {
            case hicbit_Port.port1:
                ADCPin = AnalogPin.P1;
                break;
            case hicbit_Port.port2:
                ADCPin = AnalogPin.P2;
                break;
            case hicbit_Port.port3:
                ADCPin = AnalogPin.P3;
                break;
            case hicbit_Port.port4:
                ADCPin = AnalogPin.P4;
                break;
        }
        let adValue = pins.analogReadPin(ADCPin);
        adValue = adValue * 255 / 1023;
        return Math.round(adValue);
    }

    let distanceBak = 0;
    /**
     * Get the distance of ultrasonic detection to the obstacle 超声波
     */
    //% weight=98 blockId=hicbit_ultrasonic  block="Ultrasonic|port %port|distance(cm)"
    export function hicbit_ultrasonic(port: hicbit_Port): number {
        let echoPin: DigitalPin;
        let trigPin: DigitalPin;
        switch (port) {
            case hicbit_Port.port1:
                echoPin = DigitalPin.P15;
                trigPin = DigitalPin.P1;
                break;
            case hicbit_Port.port2:
                echoPin = DigitalPin.P13;
                trigPin = DigitalPin.P2;
                break;
            case hicbit_Port.port3:
                echoPin = DigitalPin.P14;
                trigPin = DigitalPin.P3;
                break;
            case hicbit_Port.port4:
                echoPin = DigitalPin.P10;
                trigPin = DigitalPin.P4;
                break;
        }
        pins.setPull(echoPin, PinPullMode.PullNone);
        pins.setPull(trigPin, PinPullMode.PullNone);

        pins.digitalWritePin(trigPin, 0);
        control.waitMicros(2);
        pins.digitalWritePin(trigPin, 1);
        control.waitMicros(10);
        pins.digitalWritePin(trigPin, 0);
        control.waitMicros(5);
        let d = pins.pulseIn(echoPin, PulseValue.High, 25000);
        let distance = d;
        // filter timeout spikes
        if (distance == 0 && distanceBak != 0) {
            distance = distanceBak;
        }
        distanceBak = d;
        return Math.round(distance * 10 / 6 / 58);
    }

    /**
    * Get the ad value of the knob moudule 旋钮
    */
    //% weight=97 blockId=hicbit_getKnobValue  block="Get knob|port %port|value(0~255)"
    export function hicbit_getKnobValue(port: hicbit_Port): number {
        let ADCPin: AnalogPin;
        switch (port) {
            case hicbit_Port.port1:
                ADCPin = AnalogPin.P1;
                break;
            case hicbit_Port.port2:
                ADCPin = AnalogPin.P2;
                break;
            case hicbit_Port.port3:
                ADCPin = AnalogPin.P3;
                break;
            case hicbit_Port.port4:
                ADCPin = AnalogPin.P4;
                break;
        }
        let adValue = pins.analogReadPin(ADCPin);
        adValue = adValue * 255 / 1023;
        return Math.round(adValue);
    }

    /**
    * Get the ad value of the photosensitive moudule 光敏AD
    */
    //% weight=96 blockId=hicbit_getphotosensitiveValue  block="Get Photosensitive|port %port|value(0~255)"
    export function hicbit_getphotosensitiveValue(port: hicbit_Port): number {
        let ADCPin: AnalogPin;
        switch (port) {
            case hicbit_Port.port1:
                ADCPin = AnalogPin.P1;
                break;
            case hicbit_Port.port2:
                ADCPin = AnalogPin.P2;
                break;
            case hicbit_Port.port3:
                ADCPin = AnalogPin.P3;
                break;
            case hicbit_Port.port4:
                ADCPin = AnalogPin.P4;
                break;
        }
        let adValue = pins.analogReadPin(ADCPin);
        adValue = adValue * 255 / 1023;
        return 255 - Math.round(adValue);
    }

    /**
    * Get the Photosensitive sensor status,1 detect bright,0 no detect bright 光敏
    */
    //% weight=95 blockId=hicbit_photosensitiveSensor block="Photosensitive sensor|port %port|detect bright"
    export function hicbit_photosensitiveSensor(port: hicbit_Port): boolean {
        let status = 0;
        let flag: boolean = false;
        switch (port) {
            case hicbit_Port.port1:
                pins.setPull(DigitalPin.P15, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P15);
                break;
            case hicbit_Port.port2:
                pins.setPull(DigitalPin.P13, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P13);
                break;
            case hicbit_Port.port3:
                pins.setPull(DigitalPin.P14, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P14);
                break;
            case hicbit_Port.port4:
                pins.setPull(DigitalPin.P10, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P10);
                break;
        }
        if (status == 1)
            flag = false;
        else
            flag = true;
        return flag;
    }

    /**
    * Get the ad value of the avoid Sensor moudule 避障AD
    */
    //% weight=94 blockId=hicbit_getavoidSensorValue  block="Get avoid Sensor Value|port %port|value(0~255)"
    export function hicbit_getavoidSensorValue(port: hicbit_Port): number {
        let ADCPin: AnalogPin;
        switch (port) {
            case hicbit_Port.port1:
                ADCPin = AnalogPin.P1;
                break;
            case hicbit_Port.port2:
                ADCPin = AnalogPin.P2;
                break;
            case hicbit_Port.port3:
                ADCPin = AnalogPin.P3;
                break;
            case hicbit_Port.port4:
                ADCPin = AnalogPin.P4;
                break;
        }
        let adValue = pins.analogReadPin(ADCPin);
        adValue = adValue * 255 / 1023;
        return Math.round(adValue);
    }

    /**
    * Get the obstacle avoidance sensor status,1 detect obstacle,0 no detect obstacle 避障判断
    */
    //% weight=93 blockId=hicbit_avoidSensor block="Obstacle avoidance sensor|port %port|detect obstacle"
    export function hicbit_avoidSensor(port: hicbit_Port): boolean {
        let status = 0;
        let flag: boolean = false;
        switch (port) {
            case hicbit_Port.port1:
                pins.setPull(DigitalPin.P15, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P15);
                break;
            case hicbit_Port.port2:
                pins.setPull(DigitalPin.P13, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P13);
                break;
            case hicbit_Port.port3:
                pins.setPull(DigitalPin.P14, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P14);
                break;
                // if (P14_ad > 0xA)
                //     status = 1
                // else
                //     status = 0;
                // break;
            case hicbit_Port.port4:
                pins.setPull(DigitalPin.P10, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P10);
                break;
        }
        if (status == 1)
            flag = false;
        else
            flag = true;
        return flag;
    }

    /**
    * Get the ad value of the Sound sensor moudule 声音AD
    */
    //% weight=92 blockId=hicbit_getSoundsensorValue  block="Get Sound sensor Value|port %port|value(0~255)"
    export function hicbit_getSoundsensorValue(port: hicbit_Port): number {
        let ADCPin: AnalogPin;
        switch (port) {
            case hicbit_Port.port1:
                ADCPin = AnalogPin.P1;
                break;
            case hicbit_Port.port2:
                ADCPin = AnalogPin.P2;
                break;
            case hicbit_Port.port3:
                ADCPin = AnalogPin.P3;
                break;
            case hicbit_Port.port4:
                ADCPin = AnalogPin.P4;
                break;
        }
        let adValue = pins.analogReadPin(ADCPin);
        adValue = adValue * 255 / 1023;
        return Math.round(adValue);
    }

    /**
    * Set the Sound sensor status,1 detect the sound source,0 no detect the sound source 声音
    */
    //% weight=91 blockId=hicbit_SoundSensor block="Set the Sound sensor|port %port|detect the sound source"
    export function hicbit_SoundSensor(port: hicbit_Port): boolean {
        let status = 0;
        let flag: boolean = false;
        switch (port) {
            case hicbit_Port.port1:
                pins.setPull(DigitalPin.P15, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P15);
                break;
            case hicbit_Port.port2:
                pins.setPull(DigitalPin.P13, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P13);
                break;
            case hicbit_Port.port3:
                pins.setPull(DigitalPin.P14, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P14);
                break;
            case hicbit_Port.port4:
                pins.setPull(DigitalPin.P10, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P10);
                break;
        }
        if (status == 1)
            flag = false;
        else
            flag = true;
        return flag;
    }

    /**
    * Get the collision sensor status,1 trigger,0 no trigger 碰撞
    */
    //% weight=90 blockId=hicbit_collisionsensor block="collision sensor|port %port|is trigger"    
    export function hicbit_collisionsensor(port: hicbit_Port): boolean {
        let status = 0;
        let flag: boolean = false;
        switch (port) {
            case hicbit_Port.port1:
                pins.setPull(DigitalPin.P15, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P15);
                break;
            case hicbit_Port.port2:
                pins.setPull(DigitalPin.P13, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P13);
                break;
            case hicbit_Port.port3:
                pins.setPull(DigitalPin.P14, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P14);
                break;
            case hicbit_Port.port4:
                pins.setPull(DigitalPin.P10, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P10);
                break;
        }
        if (status == 1)
            flag = false;
        else
            flag = true;
        return flag;
    }


    

    /**
     * Determine the direction of remote sensing.
     */
    //% weight=88 blockId=hicbit_Rocker1 block="Rocker|port %port| value |%value|"
    export function hicbit_Rocker1(port: hicbit_Port, value: enRocker): boolean {
        let ADCPin: AnalogPin;
        let ports: DigitalPin;
        let x;
        let y;
        let flag: boolean = false;
        let now_state = enRocker.Nostate;

        switch (port) {         
            case hicbit_Port.port1:
                ports = DigitalPin.P15;
                ADCPin = AnalogPin.P1;
                break;
            case hicbit_Port.port2:
                ports = DigitalPin.P13;
                ADCPin = AnalogPin.P2;
                break;
            case hicbit_Port.port3:
                ports = DigitalPin.P14;
                ADCPin = AnalogPin.P3;
                break;
            case hicbit_Port.port4:
                ports = DigitalPin.P10;
                ADCPin = AnalogPin.P4;
                break;
        }
        pins.digitalWritePin(ports, 0);
        x = pins.analogReadPin(ADCPin);//x轴模拟量获取
        basic.pause(10);
        pins.digitalWritePin(ports, 1);
        y = pins.analogReadPin(ADCPin);//y轴模拟量获取

        if (x < 100) // 上
        {
            now_state = enRocker.Up;
        }
        else if (x > 800) //下
        {
            now_state = enRocker.Down;
        }
        else  // 左右
        {
            if (y < 100) //右
            {
                now_state = enRocker.Left;
            }
            else if (y > 800) //左
            {
                now_state = enRocker.Right;
            }
        }
        if (now_state == value)
            flag = true;
        else
            flag = false;
        return flag;
    }

    /**
    * Get the ad value of the Electronic gyroscope moudule 电子陀螺仪AD
    */
    //% weight=89 blockId=hicbit_getGyroscopGeValue  block="Get Electronic gyroscope Angle value|port %port|value(0~360)"
    export function hicbit_getGyroscopGeValue(port: hicbit_Port): number {
        let ADCPin: AnalogPin;
        switch (port) {
            case hicbit_Port.port1:
                ADCPin = AnalogPin.P1;
                break;
            case hicbit_Port.port2:
                ADCPin = AnalogPin.P2;
                break;
            case hicbit_Port.port3:
                ADCPin = AnalogPin.P3;
                break;
            case hicbit_Port.port4:
                ADCPin = AnalogPin.P4;
                break;
        }
        let adValue = pins.analogReadPin(ADCPin);
        adValue = adValue * 360 / 1023;
        return Math.round(adValue);
    }

    /**
     * get dht11 temperature and humidity Value
     **/
    //% weight=87 blockId="hicbit_getDHT11value" block="DHT11 set port %port|get %dhtResult"
    export function hicbit_getDHT11value(port: hicbit_Port,dhtResult: Dht11Result): number {
        let dht11pin: DigitalPin;
        switch (port) {
            case hicbit_Port.port1:
                dht11pin = DigitalPin.P1;
                break;
            case hicbit_Port.port2:
                dht11pin = DigitalPin.P2;
                break;
            case hicbit_Port.port3:
                dht11pin = DigitalPin.P3;
                break;
            case hicbit_Port.port4:
                dht11pin = DigitalPin.P4;
                break;
        
        }
        pins.digitalWritePin(dht11pin, 0);
        basic.pause(18);
        let i = pins.digitalReadPin(dht11pin);
        pins.setPull(dht11pin, PinPullMode.PullUp);
        switch (dhtResult) {
            case 0:
                let dhtvalue1 = 0;
                let dhtcounter1 = 0;
                while (pins.digitalReadPin(dht11pin) == 1);
                while (pins.digitalReadPin(dht11pin) == 0);
                while (pins.digitalReadPin(dht11pin) == 1);
                for (let i = 0; i <= 32 - 1; i++) {
                    while (pins.digitalReadPin(dht11pin) == 0)
                        dhtcounter1 = 0;
                    while (pins.digitalReadPin(dht11pin) == 1) {
                        dhtcounter1 += 1;
                    }
                    if (i > 15) {
                        if (dhtcounter1 > 2) {
                            dhtvalue1 = dhtvalue1 + (1 << (31 - i));
                        }
                    }
                }
                return ((dhtvalue1 & 0x0000ff00) >> 8);
                break;
            case 1:
                while (pins.digitalReadPin(dht11pin) == 1);
                while (pins.digitalReadPin(dht11pin) == 0);
                while (pins.digitalReadPin(dht11pin) == 1);
                let dhtvalue = 0;
                let dhtcounter = 0;
                for (let i = 0; i <= 32 - 1; i++) {
                    while (pins.digitalReadPin(dht11pin) == 0)
                        dhtcounter = 0;
                    while (pins.digitalReadPin(dht11pin) == 1) {
                        dhtcounter += 1;
                    }
                    if (i > 15) {
                        if (dhtcounter > 2) {
                            dhtvalue = dhtvalue + (1 << (31 - i));
                        }
                    }
                }
                return Math.round((((dhtvalue & 0x0000ff00) >> 8) * 9 / 5) + 32);
                break;
            case 2:
                while (pins.digitalReadPin(dht11pin) == 1);
                while (pins.digitalReadPin(dht11pin) == 0);
                while (pins.digitalReadPin(dht11pin) == 1);

                let value = 0;
                let counter = 0;

                for (let i = 0; i <= 8 - 1; i++) {
                    while (pins.digitalReadPin(dht11pin) == 0)
                        counter = 0;
                    while (pins.digitalReadPin(dht11pin) == 1) {
                        counter += 1;
                    }
                    if (counter > 3) {
                        value = value + (1 << (7 - i));
                    }
                }
                return value;
            default:
                return 0;
        }
        
    }

}

/**
 * IR remote
 */
//% color=50 weight=7
//% icon="\uf1eb"
namespace IR {

    // export enum hicbit_Port_IR {
    //     //% block="port 1"
    //     port1 = 21,
    //     //% block="port 2"
    //     port2 = 23,
    //     //% block="port 3"
    //     port3 = 22,
    //     //% block="port 4"
    //     port4 = 6,
    // }

    /**
    * initialization
    */
    //% blockId=ir_init
    //% blockGap=20 weight=90
    //% block="connect ir receiver to %pin"
    //% shim=IR::init
    export function init(pin: hicbit_Port_IR): void {
        return
    }
    
    /**
    * button pushed.
    */
    //% blockId=ir_received_event
    //% blockGap=20 weight=70
    //% block="on |%btn| button pressed"
    //% shim=IR::onPressEvent
    export function onPressEvent(btn: RemoteButton, body:Action): void {
        return
    }
    
}

/**
 * RGB light
 */
//% color=#CD9B9B weight=6
//% icon="\uf0eb"
namespace RGB_light {

    export enum hicbit_Port {
        //% block="port A"
        port1 = 0x01,
        //% block="port B"
        port2 = 0x02,
        //% block="port C"
        port3 = 0x03,
        //% block="Port D"
        port4 = 0x04
    }

    let lhRGBLight: hicbitRGBLight.LHhicbitRGBLight;
    /**
	 * Initialize Light belt
	 */
    //% weight=100 blockId=hicbit_initRGBLight block="Initialize light belt at port %port"
    export function hicbit_initRGBLight(port: hicbit_Port) {
        switch (port) {
            case hicbit_Port.port1:
                if (!lhRGBLight) {
                    lhRGBLight = hicbitRGBLight.create(DigitalPin.P15, 3, hicbitRGBPixelMode.RGB);
                }
                break;
            case hicbit_Port.port2:
                if (!lhRGBLight) {
                    lhRGBLight = hicbitRGBLight.create(DigitalPin.P13, 3, hicbitRGBPixelMode.RGB);
                }
                break;
            case hicbit_Port.port3:
                if (!lhRGBLight) {
                    lhRGBLight = hicbitRGBLight.create(DigitalPin.P14, 3, hicbitRGBPixelMode.RGB);
                }
                break;
            case hicbit_Port.port4:
                if (!lhRGBLight) {
                    lhRGBLight = hicbitRGBLight.create(DigitalPin.P10, 3, hicbitRGBPixelMode.RGB);
                }
                break;
        }
        lhRGBLight.clear();
    }
    
    /**
	 * Set RGB
	 */
    //% weight=99 blockId=hicbit_setPixelRGB block="Set light belt at|%lightoffset|color to |red %red|and|green %green|and|blue %blue|"
    //% inlineInputMode=inline
    //% red.min=0 red.max=255
    //% green.min=0 green.max=255
    //% blue.min=0 blue.max=255
    export function hicbit_setPixelRGB(lightoffset: hicbitLight, red: number, green: number, blue: number) {
        if (lightoffset == lhRGBLight._length)//全部
        {
            for (let i = 0; i < lhRGBLight._length; i++)
            {
                lhRGBLight.RGB(i, red, green, blue);     
            }
        }
        else
        {
            lhRGBLight.RGB(lightoffset, red, green, blue); 
        }
    }

    /**
     * Display the colored lights, and set the color of the colored lights to match the use. After setting the color of the colored lights, the color of the lights must be displayed.
     */
    //% weight=98 blockId=hicbit_showLight block="Show light belt"
    export function hicbit_showLight() {
        lhRGBLight.show();
    }

    /**
     * Clear the color of the colored lights and turn off the lights.
     */
    //% weight=97 blockGap=20 blockId=hicbit_clearLight block="Clear light"
    export function hicbit_clearLight() {
        lhRGBLight.clear();
    }
    
}


/*
 Display package
*/
//% weight=5 icon="\uf108" color=#6E8B3D
namespace Display {

    export let NEW_LINE = "\r\n";

    export enum Linenum {
        //% block="first_line"
        first_line = 0x01,
        //% block="second_line"
        second_line = 0x02,
        //% block="Third_line"
        Third_line = 0x03,
        //% block="Fourth_line"
        Fourth_line = 0x04,
        //% block="Fifth_line"
        Fifth_line = 0x05,
        
    }

    export enum unit {
        //% block="none"
        none = 0x01,
        //% block="m"
        m = 0x02,
        //% block="cm"
        cm = 0x03,
        //% block="mm"
        mm = 0x04,
        //% block="C"
        C = 0x05,
        //% block="F"
        F = 0x06,
        //% block="%"
        bf = 0x07
        
    }
    
    /**
        * Display clear
        */
    //% weight=100 blockId=Clearscreen block="Clear screen"
    export function Clearscreen(): void {
        let buf = pins.createBuffer(1);
        buf[0] = 9;
        serial.writeBuffer(buf);
        serial.writeString(NEW_LINE);
        basic.pause(200);
    }

    /**
        * Display ultrasonic distance
        */
    //% weight=99 blockId=setDisplay block="Display %line |text: %text | value: %value| unit1: %unit1"
    export function setDisplay(line: Linenum, text: string, value: number = 0, unit1: unit): void {
        let num: number = 1;
        let text2: string = " ";
        let buf = pins.createBuffer(1);
        switch (line) {
            case Linenum.first_line:
                num = 4;
                break;
            case Linenum.second_line:
                num = 5;
                break;
            case Linenum.Third_line:
                num = 6;
                break;
            case Linenum.Fourth_line:
                num = 7;
                break;
            case Linenum.Fifth_line:
                num = 8;
                break;
        }
        buf[0] = num;
        serial.writeBuffer(buf);
        if (!text) text = "";
        serial.writeString(text);
        serial.writeString(value.toString());
        switch (unit1) {
            case unit.none:
                text2 = " ";
                break;
            case unit.m:
                text2 = "m";
                break;
            case unit.cm:
                text2 = "cm";
                break;
            case unit.mm:
                text2 = "mm";
                break;
            case unit.C:
                text2 = "C";
                break;
            case unit.F:
                text2 = "F";
                break;
            case unit.bf:
                text2 = "%";
                break;
            
        }
        serial.writeString(text2);
        serial.writeString(NEW_LINE);
        basic.pause(200);
    }

}
