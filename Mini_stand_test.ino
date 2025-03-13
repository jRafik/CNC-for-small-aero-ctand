/***************************Мини стенд****************************

Программа создана для управления ЧПУ маленького стенда, реализовано:
- перемещение по осям;
- замер давлений;
- запись измерений на флешку либо на COM порт;
- управление энкодором;
- вывод информации на экран.

***************************Подключение*****************************

D4 - D9 шаговики (step, dir);
D2 - D3 энкодер (D18 - Key);
D20 (SDA), D21 (SCL) экран;
D51 (MOSI), D50 (MISO), D52 (CLK), D48 (CS) флешка;
D50 (MISO), D52 (SCLK), D53 (SS) датчик 1;

*****************************Настройка****************************/


#define thread_pitch_X 2.0  //шаг резьбы винта, [мм]
#define thread_pitch_Z 2.0  //шаг резьбы винта, [мм]
#define BTN_PIN 18          //пин кнопки энкодера
#define MY_PERIOD 150       //время обновления экрана в миллисекундах (1 с = 1000 мс)
#define CLK 2               //пин энкодера
#define DT 3                //пин энкодера
#define SW 18               //пин кнопки энкодера

//**************************Библиотеки****************************

#include <SPI.h>                //подключение библиотеки протокола SPI
#include <SD.h>                 //подключение библиотеки SD карты
#include "GyverStepper2.h"      //подключение библиотеки шаговика
#include <Honeywell_SPI.h>      //подключение библиотеки датчика
#include <LiquidCrystal_I2C.h>  //подключение библиотеки экрана
#include "GyverEncoder.h"       //подключение библиотеки энкодера

//**********************Переменные перемещения********************

GStepper2< STEPPER2WIRE> stepper_X(400, 4, 7);  // драйвер step-dir
GStepper2< STEPPER2WIRE> stepper_Z(400, 6, 9);  // драйвер step-dir

float x_position = 0.0;  //позиция по X
float z_position = 0.0;  //позиция по Z

float one_measure_deg_X = 0.0;  //шаг одного измерения по X
float one_measure_deg_Z = 0.0;  //шаг одного измерения по Z

float one_micromeasure_deg_Z = 0.0;  //шаг одного измерения по Z

float my_deg_X = 0.0;  //угол по X
float my_deg_Z = 0.0;  //угол по Z

float z_measuring = 0.0;         //высота измерения по Z
float z_step = 0.0;              //шаг измерения по Z
float z_height_microstep = 0.0;  //высота микроизмерения по Z (для пограничного слоя)
float z_step_microstep = 0.0;    //шаг микроизмерения по Z

float x_measuring = 0.0;  //высота измерения по X
float x_step = 0.0;       //шаг измерения по X

float diameter = 0.0;
float corner = 0.0;
float radius = 0.0;
float corner_sum = 0.0;
float rad = 0.0;
float step_rad = 0.0;
float z_corect = 0.0;

//*************************Датчик давления*************************

Honeywell_SPI PS1(53, 0.915541, -6000, 1638, 10);  //создание объекта датчика
unsigned int waiting = 0;

//*************************Переменные энкодера*************************

volatile int counter = 0;          // счётчик
Encoder enc1(CLK, DT, SW, TYPE2);  //создание объекта энкодера
byte button_flag = 0;
unsigned long button_timer;

//********************Переменные режимов работы********************

int mode = 0;
int mode_along_Z = 0;
int mode_along_XZ = 0;
int mode_settings = 0;
int mode_move_XYZ = 0;
int mode_along_cyrcle_XZ = 0;
int cursor_string = 0;

//************************Переменные экрана************************

unsigned long tmr = 0;               //таймер экрана
LiquidCrystal_I2C lcd(0x27, 20, 4);  // адрес, столбцов, строк

//******************Переменные для настройки***********************

int max_speed = 1250;
int max_acceleration = 1250;

//******************Переменные для SD карты************************

const byte PIN_CHIP_SELECT = 48;
String results = " ";
byte card_flag = 0;

//*****************************************************************

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  pinMode(48, OUTPUT);
  PS1.begin();
  pinMode(18, INPUT_PULLUP);
  attachInterrupt(0, encIsr, CHANGE);
  attachInterrupt(1, encIsr, CHANGE);
  attachInterrupt(5, isr, FALLING);
  stepper_X.setMaxSpeedDeg(max_speed);          // скорость движения к цели x
  stepper_X.setAcceleration(max_acceleration);  // ускорение x
  stepper_Z.setMaxSpeedDeg(max_speed);          // скорость движения к цели z
  stepper_Z.setAcceleration(max_acceleration);  // ускорение z
}

void loop() {
  switch (mode) {
    case 0:  //основное меню
      if (millis() - tmr >= MY_PERIOD) {
        tmr = millis();
        cursor();
        cursor_string = constrain(cursor_string, 0, 2);
        menu_main();
      }
      if (button_flag == 1 && cursor_string == 0) {
        mode = 1;
        cursor_string = 0;
        button_flag = 0;
      }
      if (button_flag == 1 && cursor_string == 1) {
        mode = 2;
        cursor_string = 0;
        button_flag = 0;
      }
      if (button_flag == 1 && cursor_string == 2) {
        mode = 3;
        cursor_string = 0;
        button_flag = 0;
      }
      break;

    case 1:  //меню режимов измерений
      if (millis() - tmr >= MY_PERIOD) {
        tmr = millis();
        cursor();
        cursor_string = constrain(cursor_string, 0, 3);
        menu_measurings();
      }
      if (button_flag == 1 && cursor_string == 0) {
        mode = 0;
        cursor_string = 0;
        button_flag = 0;
      }
      if (button_flag == 1 && cursor_string == 1) {
        mode = 4;
        cursor_string = 0;
        button_flag = 0;
      }
      if (button_flag == 1 && cursor_string == 2) {
        mode = 5;
        cursor_string = 0;
        button_flag = 0;
      }
      if (button_flag == 1 && cursor_string == 3) {
        mode = 6;
        cursor_string = 0;
        button_flag = 0;
      }
      break;

    case 2:  //меню перемещения по осям
      switch (mode_move_XYZ) {
        case 0:
          if (millis() - tmr >= MY_PERIOD) {  // ищем разницу
            tmr = millis();
            cursor();
            cursor_string = constrain(cursor_string, 0, 2);
            menu_moveXYZ();
          }
          if (button_flag == 1 && cursor_string == 0) {
            mode = 0;
            cursor_string = 1;
            button_flag = 0;
          }
          if (button_flag == 1 && cursor_string == 1) {
            mode_move_XYZ = 1;
            cursor_string = 0;
            button_flag = 0;
          }
          if (button_flag == 1 && cursor_string == 2) {
            mode_move_XYZ = 3;
            cursor_string = 0;
            button_flag = 0;
          }
          break;

        case 1:  //меню перемещение по X
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            cursor();
            cursor_string = constrain(cursor_string, 0, 3);
            menu_move();
          }
          if (button_flag == 1 && cursor_string == 0) {
            mode_move_XYZ = 0;
            cursor_string = 1;
            button_flag = 0;
          }
          if (button_flag == 1 && cursor_string == 1) {
            mode_move_XYZ = 4;
            button_flag = 0;
          }
          if (button_flag == 1 && cursor_string == 2) {
            mode_move_XYZ = 5;
            button_flag = 0;
          }
          if (button_flag == 1 && cursor_string == 3) {
            mode_move_XYZ = 6;
            button_flag = 0;
          }
          break;

        case 3:  //перемещение по Z
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            cursor();
            cursor_string = constrain(cursor_string, 0, 3);
            menu_move();
          }
          if (button_flag == 1 && cursor_string == 0) {
            mode_move_XYZ = 0;
            cursor_string = 3;
            button_flag = 0;
          }
          if (button_flag == 1 && cursor_string == 1) {
            mode_move_XYZ = 10;
            button_flag = 0;
          }
          if (button_flag == 1 && cursor_string == 2) {
            mode_move_XYZ = 11;
            button_flag = 0;
          }
          if (button_flag == 1 && cursor_string == 3) {
            mode_move_XYZ = 12;
            button_flag = 0;
          }
          break;

        case 4:  //перемещение по X на 10 мм
          x_position += counter * 10.0;
          my_deg_X += counter * 1800.0;
          counter = 0;
          move_X();
          lcd_print_x();
          if (button_flag == 1) {
            button_flag = 0;
            stepper_X.brake();
            mode_move_XYZ = 1;
            my_deg_X = 0.0;
            x_position = 0.0;
            stepper_X.setCurrent(0);
          }
          break;

        case 5:  //перемещение по X на 1 мм
          x_position += counter * 1.0;
          my_deg_X += counter * 180.0;
          counter = 0;
          move_X();
          lcd_print_x();
          if (button_flag == 1) {
            button_flag = 0;
            stepper_X.brake();
            mode_move_XYZ = 1;
            my_deg_X = 0.0;
            x_position = 0.0;
            stepper_X.setCurrent(0);
          }
          break;

        case 6:  //перемещение по X на 0,1 мм
          x_position += counter * 0.1;
          my_deg_X += counter * 18.0;
          counter = 0;
          move_X();
          lcd_print_x();
          if (button_flag == 1) {
            button_flag = 0;
            stepper_X.brake();
            mode_move_XYZ = 1;
            my_deg_X = 0.0;
            x_position = 0.0;
            stepper_X.setCurrent(0);
          }
          break;

        case 10:  //перемещение по Z на 10 мм
          z_position += counter * 10.0;
          my_deg_Z += counter * 1800.0;
          counter = 0;
          move_Z();
          lcd_print_z();
          if (button_flag == 1) {
            button_flag = 0;
            stepper_Z.brake();
            mode_move_XYZ = 3;
            my_deg_Z = 0.0;
            z_position = 0.0;
            stepper_Z.setCurrent(0);
          }
          break;

        case 11:  //перемещение по Z на 1 мм
          z_position += counter * 1.0;
          my_deg_Z += counter * 180.0;
          counter = 0;
          move_Z();
          lcd_print_z();
          if (button_flag == 1) {
            button_flag = 0;
            stepper_Z.brake();
            mode_move_XYZ = 3;
            my_deg_Z = 0.0;
            z_position = 0.0;
            stepper_Z.setCurrent(0);
          }
          break;

        case 12:  //перемещение по Z на 0,1 мм
          z_position += counter * 0.1;
          my_deg_Z += counter * 18.0;
          counter = 0;
          move_Z();
          lcd_print_z();
          if (button_flag == 1) {
            button_flag = 0;
            stepper_Z.brake();
            mode_move_XYZ = 3;
            my_deg_Z = 0.0;
            z_position = 0.0;
            stepper_Z.setCurrent(0);
          }
          break;
      }
      break;

    case 3:  //меню настроек
      switch (mode_settings) {
        case 0:
          if (millis() - tmr >= MY_PERIOD) {  // ищем разницу
            tmr = millis();
            cursor();
            cursor_string = constrain(cursor_string, 0, 2);
            menu_settings();
          }
          if (button_flag == 1 && cursor_string == 0) {
            button_flag = 0;
            mode = 0;
            cursor_string = 2;
          }
          if (button_flag == 1 && cursor_string == 1) {
            button_flag = 0;
            mode_settings = 1;
          }
          if (button_flag == 1 && cursor_string == 2) {
            button_flag = 0;
            mode_settings = 2;
          }
          break;

        case 1:
          if (button_flag == 1) {
            button_flag = 0;
            mode_settings = 0;
            cursor_string = 1;
            stepper_X.setMaxSpeedDeg(max_speed);  // скорость движения к цели x
            stepper_Z.setMaxSpeedDeg(max_speed);  // скорость движения к цели z
          }
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            max_speed += counter * 50;
            if (max_speed < 100) max_speed = 100;
            counter = 0;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Max speed:");
            lcd.setCursor(0, 1);
            lcd.print(max_speed);
            lcd.setCursor(8, 1);
            lcd.print("deg/s");
          }
          break;

        case 2:
          if (button_flag == 1) {
            button_flag = 0;
            mode_settings = 0;
            cursor_string = 2;
            stepper_X.setAcceleration(max_acceleration);  // ускорение x
            stepper_Z.setAcceleration(max_acceleration);  // ускорение z
          }
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            max_acceleration += counter * 50;
            if (max_acceleration < 100) max_acceleration = 100;
            counter = 0;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Max acceleration:");
            lcd.setCursor(0, 1);
            lcd.print(max_acceleration);
            lcd.setCursor(8, 1);
            lcd.print("step/s^2");
          }
          break;
      }
      break;

    case 4:  //измерение по оси Z
      switch (mode_along_Z) {
        case 0:
          if (millis() - tmr >= MY_PERIOD) {  // ищем разницу
            tmr = millis();
            cursor();
            cursor_string = constrain(cursor_string, 0, 3);
            menu_along_Z_1();
          }
          if (button_flag == 1 && cursor_string == 0) {
            button_flag = 0;
            mode = 1;
            cursor_string = 1;
          }
          if (button_flag == 1 && cursor_string == 1) {
            button_flag = 0;
            mode_along_Z = 1;
          }
          if (button_flag == 1 && cursor_string == 2) {
            button_flag = 0;
            mode_along_Z = 2;
          }
          if (button_flag == 1 && cursor_string == 3) {
            button_flag = 0;
            mode_along_Z = 3;
            cursor_string = 0;
          }
          break;

        case 1:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_Z = 0;
            cursor_string = 1;
          }
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            z_measuring += counter * 10;
            select_z_measuring();
          }
          break;

        case 2:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_Z = 0;
            cursor_string = 2;
          }
          if (millis() - tmr >= MY_PERIOD) {  // ищем разницу
            tmr = millis();
            z_measuring += counter;
            select_z_measuring();
          }
          break;

        case 3:
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            cursor();
            cursor_string = constrain(cursor_string, 0, 3);
            menu_along_Z_2();
          }
          if (button_flag == 1 && cursor_string == 0) {
            button_flag = 0;
            mode_along_Z = 0;
            cursor_string = 3;
          }
          if (button_flag == 1 && cursor_string == 1) {
            button_flag = 0;
            mode_along_Z = 4;
          }
          if (button_flag == 1 && cursor_string == 2) {
            button_flag = 0;
            mode_along_Z = 5;
          }
          if (button_flag == 1 && cursor_string == 3) {
            button_flag = 0;
            mode_along_Z = 6;
            cursor_string = 0;
          }
          break;

        case 4:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_Z = 3;
            cursor_string = 1;
          }
          if (millis() - tmr >= MY_PERIOD) {  // ищем разницу
            tmr = millis();
            select_z_step();
          }
          break;

        case 5:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_Z = 3;
            cursor_string = 2;
          }
          if (millis() - tmr >= MY_PERIOD) {  // ищем разницу
            tmr = millis();
            select_waiting();
          }
          break;

        case 6:
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            cursor();
            cursor_string = constrain(cursor_string, 0, 3);
            menu_along_Z_3();
          }
          if (button_flag == 1 && cursor_string == 0) {
            button_flag = 0;
            mode_along_Z = 3;
            cursor_string = 3;
          }
          if (button_flag == 1 && cursor_string == 1) {
            button_flag = 0;
            mode_along_Z = 7;
          }
          if (button_flag == 1 && cursor_string == 2) {
            button_flag = 0;
            mode_along_Z = 8;
          }
          if (button_flag == 1 && cursor_string == 3) {
            button_flag = 0;
            mode_along_Z = 9;
          }
          break;

        case 7:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_Z = 6;
            cursor_string = 1;
          }
          if (millis() - tmr >= MY_PERIOD) {  // ищем разницу
            tmr = millis();
            select_z_height_microstep();
          }
          break;

        case 8:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_Z = 6;
            cursor_string = 2;
          }
          if (millis() - tmr >= MY_PERIOD) {  // ищем разницу
            tmr = millis();
            select_z_step_microstep();
          }
          break;

        case 9:
          stepper_X.reset();
          stepper_Z.reset();
          one_measure_deg_Z = z_step / (thread_pitch_Z / 360.0);  //расчёт на сколько градусов повернется вал мотора между двумя измерениями
          one_micromeasure_deg_Z = z_step_microstep / (thread_pitch_Z / 360.0);
          mode_along_Z = 10;
          break;

        case 10:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_Z = 12;
          }
          stepper_Z.setTargetDeg(my_deg_Z, ABSOLUTE);
          if (stepper_Z.ready() == 0) {
            stepper_Z.tick();
          } else {
            delay(waiting);
            sensor();
            write_to_card();
            measuring_results();
            mode_along_Z = 11;
          }
          break;

        case 11:
          if (z_position > z_measuring) {
            mode_along_Z = 12;
          } else if ((z_position < z_height_microstep) || (z_position >= (z_measuring - z_height_microstep))) {
            my_deg_Z += one_micromeasure_deg_Z;
            z_position += z_step_microstep;
            mode_along_Z = 10;
          } else {
            my_deg_Z += one_measure_deg_Z;
            z_position += z_step;
            mode_along_Z = 10;
          }

          break;

        case 12:
          stepper_Z.setTargetDeg(0.0, ABSOLUTE);
          if (stepper_Z.ready() == 0) {
            stepper_Z.tick();
          } else {
            z_position = 0;
            my_deg_Z = 0;
            card_flag = 0;
            mode_along_Z = 6;
          }
          break;
      }
      break;

    case 5:  //измерение по плоскости XZ
      switch (mode_along_XZ) {
        case 0:
          if (millis() - tmr >= MY_PERIOD) {  // ищем разницу
            tmr = millis();
            cursor();
            cursor_string = constrain(cursor_string, 0, 3);
            menu_along_Z_1();
          }
          if (button_flag == 1 && cursor_string == 0) {
            button_flag = 0;
            mode = 1;
            cursor_string = 2;
          }
          if (button_flag == 1 && cursor_string == 1) {
            button_flag = 0;
            mode_along_XZ = 1;
          }
          if (button_flag == 1 && cursor_string == 2) {
            button_flag = 0;
            mode_along_XZ = 2;
          }
          if (button_flag == 1 && cursor_string == 3) {
            button_flag = 0;
            mode_along_XZ = 3;
            cursor_string = 0;
          }
          break;

        case 1:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_XZ = 0;
            cursor_string = 1;
          }
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            z_measuring += counter * 10;
            select_z_measuring();
          }
          break;

        case 2:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_XZ = 0;
            cursor_string = 2;
          }
          if (millis() - tmr >= MY_PERIOD) {  // ищем разницу
            tmr = millis();
            z_measuring += counter;
            select_z_measuring();
          }
          break;

        case 3:
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            cursor();
            cursor_string = constrain(cursor_string, 0, 3);
            menu_along_Z_2();
          }
          if (button_flag == 1 && cursor_string == 0) {
            button_flag = 0;
            mode_along_XZ = 0;
            cursor_string = 3;
          }
          if (button_flag == 1 && cursor_string == 1) {
            button_flag = 0;
            mode_along_XZ = 4;
          }
          if (button_flag == 1 && cursor_string == 2) {
            button_flag = 0;
            mode_along_XZ = 5;
          }
          if (button_flag == 1 && cursor_string == 3) {
            button_flag = 0;
            mode_along_XZ = 9;
            cursor_string = 0;
          }
          break;

        case 4:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_XZ = 3;
            cursor_string = 1;
          }
          if (millis() - tmr >= MY_PERIOD) {  // ищем разницу
            tmr = millis();
            select_z_step();
          }
          break;

        case 5:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_XZ = 3;
            cursor_string = 2;
          }
          if (millis() - tmr >= MY_PERIOD) {  // ищем разницу
            tmr = millis();
            select_waiting();
          }
          break;

        case 6:
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            cursor();
            cursor_string = constrain(cursor_string, 0, 3);
            menu_along_XZ_1();
          }
          if (button_flag == 1 && cursor_string == 0) {
            button_flag = 0;
            mode_along_XZ = 3;
            cursor_string = 3;
          }
          if (button_flag == 1 && cursor_string == 1) {
            button_flag = 0;
            mode_along_XZ = 7;
          }
          if (button_flag == 1 && cursor_string == 2) {
            button_flag = 0;
            mode_along_XZ = 8;
          }
          if (button_flag == 1 && cursor_string == 3) {
            button_flag = 0;
            mode_along_XZ = 9;
            cursor_string = 0;
          }
          break;

        case 7:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_XZ = 6;
            cursor_string = 1;
          }
          if (millis() - tmr >= MY_PERIOD) {  // ищем разницу
            tmr = millis();
            select_z_height_microstep();
          }
          break;

        case 8:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_XZ = 6;
            cursor_string = 2;
          }
          if (millis() - tmr >= MY_PERIOD) {  // ищем разницу
            tmr = millis();
            select_z_step_microstep();
          }
          break;

        case 9:
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            cursor();
            cursor_string = constrain(cursor_string, 0, 3);
            menu_along_XZ_2();
          }
          if (button_flag == 1 && cursor_string == 0) {
            button_flag = 0;
            mode_along_XZ = 3;
            cursor_string = 3;
          }
          if (button_flag == 1 && cursor_string == 1) {
            button_flag = 0;
            mode_along_XZ = 10;
          }
          if (button_flag == 1 && cursor_string == 2) {
            button_flag = 0;
            mode_along_XZ = 11;
          }
          if (button_flag == 1 && cursor_string == 3) {
            button_flag = 0;
            mode_along_XZ = 12;
            cursor_string = 0;
          }
          break;

        case 10:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_XZ = 9;
            cursor_string = 1;
          }
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            x_measuring += counter * 10;
            select_x_measuring();
          }
          break;

        case 11:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_XZ = 9;
            cursor_string = 1;
          }
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            x_measuring += counter * 1;
            select_x_measuring();
          }
          break;

        case 12:
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            cursor();
            cursor_string = constrain(cursor_string, 0, 2);
            menu_along_XZ_3();
          }
          if (button_flag == 1 && cursor_string == 0) {
            button_flag = 0;
            mode_along_XZ = 9;
            cursor_string = 3;
          }
          if (button_flag == 1 && cursor_string == 1) {
            button_flag = 0;
            mode_along_XZ = 13;
          }
          if (button_flag == 1 && cursor_string == 2) {
            button_flag = 0;
            mode_along_XZ = 14;
          }
          break;

        case 13:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_XZ = 12;
            cursor_string = 1;
          }
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            select_x_step();
          }
          break;

        case 14:
          stepper_X.reset();
          stepper_Z.reset();
          one_measure_deg_Z = z_step / (thread_pitch_Z / 360.0);  //расчёт на сколько градусов повернется вал мотора между двумя измерениями
          one_measure_deg_X = x_step / (thread_pitch_X / 360.0);  //расчёт на сколько градусов повернется вал мотора между двумя измерениями
          mode_along_XZ = 15;
          break;

        case 15:  //перемещение по Х
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_XZ = 18;
          }
          if (x_position > x_measuring) {
            x_position = 0.0;
            my_deg_X = 0.0;
            mode_along_XZ = 17;
          }
          stepper_X.setTargetDeg(my_deg_X, ABSOLUTE);
          if (stepper_X.ready() == 0) {
            stepper_X.tick();
          } else {
            delay(waiting);
            sensor();
            write_to_card();
            measuring_results();
            my_deg_X += one_measure_deg_X;
            x_position += x_step;
          }
          break;

        case 16:  //перемещение по Z
          stepper_Z.setTargetDeg(my_deg_Z, ABSOLUTE);
          if (stepper_Z.ready() == 0) {
            stepper_Z.tick();
          } else {
            mode_along_XZ = 15;
          }
          break;

        case 17:  //возврат Х в ноль и перемещение Z
          stepper_X.setTargetDeg(0.0, ABSOLUTE);
          if (stepper_X.ready() == 0) {
            stepper_X.tick();
          } else {
            /*if (z_position > z_measuring || button_flag == 1) {
              button_flag = 0;
              mode_along_XZ = 18;
            }  else {
              my_deg_Z += one_measure_deg_Z;
              z_position += z_step;
              mode_along_XZ = 16;
            }*/
            my_deg_Z += one_measure_deg_Z;
            z_position += z_step;
            mode_along_XZ = 16;
            if (z_position > z_measuring || button_flag == 1) {
              button_flag = 0;
              mode_along_XZ = 18;
            }
          }
          break;

        case 18:
          stepper_Z.setTargetDeg(0.0, ABSOLUTE);
          stepper_X.setTargetDeg(0.0, ABSOLUTE);
          if (stepper_Z.ready() == 0) {
            stepper_Z.tick();
          } else if (stepper_X.ready() == 0) {
            stepper_X.tick();
          } else {
            z_position = 0;
            my_deg_Z = 0;
            x_position = 0;
            my_deg_Z = 0;
            card_flag = 0;
            mode_along_XZ = 12;
          }
          break;
      }
      break;

    case 6:  //измерение по окружности
      switch (mode_along_cyrcle_XZ) {
        case 0:
          if (millis() - tmr >= MY_PERIOD) {  // ищем разницу
            tmr = millis();
            cursor();
            cursor_string = constrain(cursor_string, 0, 3);
            menu_along_circle();
          }
          if (button_flag == 1 && cursor_string == 0) {
            button_flag = 0;
            mode = 1;
            cursor_string = 3;
          }
          if (button_flag == 1 && cursor_string == 1) {
            button_flag = 0;
            mode_along_cyrcle_XZ = 1;
          }
          if (button_flag == 1 && cursor_string == 2) {
            button_flag = 0;
            mode_along_cyrcle_XZ = 2;
          }
          if (button_flag == 1 && cursor_string == 3) {
            button_flag = 0;
            mode_along_cyrcle_XZ = 3;
            cursor_string = 0;
          }
          break;

        case 1:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_cyrcle_XZ = 0;
            cursor_string = 1;
          }
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            diameter += counter * 10;
            select_diameter();
          }
          break;

        case 2:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_cyrcle_XZ = 0;
            cursor_string = 2;
          }
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            diameter += counter;
            select_diameter();
          }
          break;

        case 3:
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            cursor();
            cursor_string = constrain(cursor_string, 0, 3);
            menu_along_circle_2();
          }
          if (button_flag == 1 && cursor_string == 0) {
            button_flag = 0;
            mode_along_cyrcle_XZ = 0;
            cursor_string = 3;
          }
          if (button_flag == 1 && cursor_string == 1) {
            button_flag = 0;
            mode_along_cyrcle_XZ = 4;
          }
          if (button_flag == 1 && cursor_string == 2) {
            button_flag = 0;
            mode_along_cyrcle_XZ = 5;
          }
          if (button_flag == 1 && cursor_string == 3) {
            button_flag = 0;
            mode_along_cyrcle_XZ = 6;
            cursor_string = 0;
          }
          break;

        case 4:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_cyrcle_XZ = 3;
            cursor_string = 1;
          }
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            step_rad += counter * 0.1;
            select_step_diameter();
          }
          break;

        case 5:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_cyrcle_XZ = 3;
            cursor_string = 2;
          }
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            corner += counter;
            select_angle();
          }
          break;

        case 6:
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            cursor();
            cursor_string = constrain(cursor_string, 0, 2);
            menu_along_circle_3();
          }
          if (button_flag == 1 && cursor_string == 0) {
            button_flag = 0;
            mode_along_cyrcle_XZ = 3;
            cursor_string = 3;
          }
          if (button_flag == 1 && cursor_string == 1) {
            button_flag = 0;
            mode_along_cyrcle_XZ = 7;
          }
          if (button_flag == 1 && cursor_string == 2) {
            button_flag = 0;
            mode_along_cyrcle_XZ = 8;
          }
          break;

        case 7:
          if (button_flag == 1) {
            button_flag = 0;
            mode_along_cyrcle_XZ = 6;
            cursor_string = 1;
          }
          if (millis() - tmr >= MY_PERIOD) {
            tmr = millis();
            select_waiting();
          }
          break;

        case 8:
          stepper_X.setCurrent(0);
          stepper_Z.setCurrent(0);
          radius = diameter / 2.0;
          mode_along_cyrcle_XZ = 10;
          break;

        case 9:
          if (button_flag == 1 || radius <= 0) {
            button_flag = 0;
            cursor_string = 0;
            x_position = 0.0;
            z_position = 0.0;
            my_deg_X = 0.0;
            my_deg_Z = 0.0;
            corner_sum = 0.0;
            stepper_Z.setTargetDeg(0.0, ABSOLUTE);
            stepper_X.setTargetDeg(0.0, ABSOLUTE);
            mode_along_cyrcle_XZ = 11;
          }
          stepper_Z.setTargetDeg(my_deg_Z, ABSOLUTE);
          stepper_X.setTargetDeg(my_deg_X, ABSOLUTE);
          stepper_Z.tick();
          stepper_X.tick();
          if (stepper_Z.ready() == 1 && stepper_X.ready() == 1) {
            delay(waiting);
            sensor();
            write_to_card();
            mode_along_cyrcle_XZ = 10;
          }
          break;

        case 10:
          measuring_results_circle();
          if (corner_sum >= 360.0) {
            corner_sum = 0;
            radius = radius - step_rad;
          }
          rad = corner_sum * 0.0174533;
          x_position = radius * sin(rad);
          z_position = radius * cos(rad);
          my_deg_X = x_position / (thread_pitch_X / 360.0);
          my_deg_Z = z_position / (thread_pitch_Z / 360.0);
          corner_sum += corner;
          mode_along_cyrcle_XZ = 9;
          break;

        case 11:
          stepper_Z.tick();
          stepper_X.tick();
          if (stepper_X.getStatus() == 0 && stepper_Z.getStatus() == 0) {
            cursor_string = 2;
            card_flag = 0;
            mode_along_cyrcle_XZ = 6;
          }
          break;
      }
      break;
  }
}

void sensor() {
  PS1.readSensor();
  /*Serial.print("Pressure: ");
  Serial.print(PS1.getPressure());
  Serial.print(" Pa; Z: ");
  Serial.print(z_position);
  Serial.print(" mm; X: ");
  Serial.print(x_position);
  Serial.println(" mm.");*/
}

void measuring_results() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("X:");
  lcd.setCursor(2, 0);
  lcd.print(x_position);
  lcd.setCursor(10, 0);
  lcd.print("/");
  lcd.setCursor(11, 0);
  lcd.print(x_measuring);
  lcd.setCursor(18, 0);
  lcd.print("mm");

  lcd.setCursor(0, 1);
  lcd.print("Z:");
  lcd.setCursor(2, 1);
  lcd.print(z_position);
  lcd.setCursor(10, 1);
  lcd.print("/");
  lcd.setCursor(11, 1);
  lcd.print(z_measuring);
  lcd.setCursor(18, 1);
  lcd.print("mm");

  lcd.setCursor(0, 2);
  lcd.print("Pt:");
  lcd.setCursor(3, 2);
  lcd.print(PS1.getPressure());
  lcd.setCursor(18, 2);
  lcd.print("Pa");
}

void measuring_results_circle() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("D:");
  lcd.setCursor(2, 0);
  lcd.print(radius * 2);
  lcd.setCursor(10, 0);
  lcd.print("/");
  lcd.setCursor(11, 0);
  lcd.print(diameter);
  lcd.setCursor(18, 0);
  lcd.print("mm");

  lcd.setCursor(0, 1);
  lcd.print("Angle:");
  lcd.setCursor(6, 1);
  lcd.print(corner_sum);
  lcd.setCursor(19, 1);
  lcd.write(223);

  lcd.setCursor(0, 2);
  lcd.print("Pt:");
  lcd.setCursor(3, 2);
  lcd.print(PS1.getPressure());
  lcd.setCursor(18, 2);
  lcd.print("Pa");
}

void lcd_print_x() {
  if (millis() - tmr >= MY_PERIOD) {
    tmr = millis();
    if (stepper_X.getStatus() == 0) {
      PS1.readSensor();
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("X position:");
    lcd.setCursor(0, 1);
    lcd.print(x_position);
    lcd.setCursor(8, 1);
    lcd.print("mm");
    lcd.setCursor(0, 2);
    lcd.print("Pressure:");
    lcd.setCursor(0, 3);
    lcd.print(PS1.getPressure());
    lcd.setCursor(10, 3);
    lcd.print("Pa");
  }
}

void move_X() {
  stepper_X.setTargetDeg(my_deg_X, ABSOLUTE);
  if (stepper_X.ready() == 0) {
    stepper_X.tick();
  }
}

void lcd_print_z() {
  if (millis() - tmr >= MY_PERIOD) {
    tmr = millis();
    if (stepper_Z.getStatus() == 0) {
      PS1.readSensor();
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Z position:");
    lcd.setCursor(0, 1);
    lcd.print(z_position);
    lcd.setCursor(8, 1);
    lcd.print("mm");
    lcd.setCursor(0, 2);
    lcd.print("Pressure:");
    lcd.setCursor(0, 3);
    lcd.print(PS1.getPressure());
    lcd.setCursor(10, 3);
    lcd.print("Pa");
  }
}

void move_Z() {
  stepper_Z.setTargetDeg(my_deg_Z, ABSOLUTE);
  if (stepper_Z.ready() == 0) {
    stepper_Z.tick();
  }
}

void menu_along_circle() {
  lcd.clear();
  lcd.setCursor(0, cursor_string);
  lcd.write(126);
  lcd.setCursor(1, 0);  // столбец 1 строка 0
  lcd.print("Back");
  lcd.setCursor(1, 1);  // столбец 1 строка 0
  lcd.print("Diameter 10 mm");
  lcd.setCursor(1, 2);
  lcd.print("Diameter 1 mm");
  lcd.setCursor(1, 3);
  lcd.print("Next");
}

void menu_along_circle_2() {
  lcd.clear();
  lcd.setCursor(0, cursor_string);
  lcd.write(126);
  lcd.setCursor(1, 0);  // столбец 1 строка 0
  lcd.print("Back");
  lcd.setCursor(1, 1);  // столбец 1 строка 0
  lcd.print("Step");
  lcd.setCursor(1, 2);
  lcd.print("Angle");
  lcd.setCursor(1, 3);
  lcd.print("Next");
}

void menu_along_circle_3() {
  lcd.clear();
  lcd.setCursor(0, cursor_string);
  lcd.write(126);
  lcd.setCursor(1, 0);  // столбец 1 строка 0
  lcd.print("Back");
  lcd.setCursor(1, 1);  // столбец 1 строка 0
  lcd.print("Waiting");
  lcd.setCursor(1, 2);
  lcd.print("Start");
}

void menu_settings() {
  lcd.clear();
  lcd.setCursor(0, cursor_string);
  lcd.write(126);
  lcd.setCursor(1, 0);  // столбец 1 строка 0
  lcd.print("Back");
  lcd.setCursor(1, 1);  // столбец 1 строка 0
  lcd.print("Motor speeds");
  lcd.setCursor(1, 2);
  lcd.print("Motor acceleration");
}

void menu_moveXYZ() {
  lcd.clear();
  lcd.setCursor(0, cursor_string);
  lcd.write(126);
  lcd.setCursor(1, 0);  // столбец 1 строка 0
  lcd.print("Back");
  lcd.setCursor(1, 1);  // столбец 1 строка 0
  lcd.print("Move X");
  lcd.setCursor(1, 2);
  lcd.print("Move Z");
}

void menu_move() {
  lcd.clear();
  lcd.setCursor(0, cursor_string);
  lcd.write(126);
  lcd.setCursor(1, 0);  // столбец 1 строка 0
  lcd.print("Back");
  lcd.setCursor(1, 1);  // столбец 1 строка 0
  lcd.print("10 mm");
  lcd.setCursor(1, 2);
  lcd.print("1 mm");
  lcd.setCursor(1, 3);
  lcd.print("0.1 mm");
}

void menu_measurings() {

  lcd.clear();
  lcd.setCursor(0, cursor_string);
  lcd.write(126);
  lcd.setCursor(1, 0);  // столбец 1 строка 0
  lcd.print("Back");
  lcd.setCursor(1, 1);  // столбец 1 строка 0
  lcd.print("Along axis Z");
  lcd.setCursor(1, 2);
  lcd.print("Along plane XZ");
  lcd.setCursor(1, 3);
  lcd.print("Along circle XZ");
}

void menu_main() {
  lcd.clear();
  lcd.setCursor(0, cursor_string);
  lcd.write(126);
  lcd.setCursor(1, 0);  // столбец 1 строка 0
  lcd.print("Measuring");
  lcd.setCursor(1, 1);
  lcd.print("Move XYZ");
  lcd.setCursor(1, 2);
  lcd.print("Settings");
  if (SD.begin(PIN_CHIP_SELECT)) {
    lcd.setCursor(2, 3);
    lcd.print("Card initialized");
  }
}

void menu_along_Z_1() {
  lcd.clear();
  lcd.setCursor(0, cursor_string);
  lcd.write(126);
  lcd.setCursor(1, 0);  // столбец 1 строка 0
  lcd.print("Back");
  lcd.setCursor(1, 1);  // столбец 1 строка 0
  lcd.print("Z 10 mm");
  lcd.setCursor(1, 2);
  lcd.print("Z 1 mm");
  lcd.setCursor(1, 3);
  lcd.print("Next");
}

void menu_along_Z_2() {
  lcd.clear();
  lcd.setCursor(0, cursor_string);
  lcd.write(126);
  lcd.setCursor(1, 0);
  lcd.print("Back");
  lcd.setCursor(1, 1);
  lcd.print("Z Step");
  lcd.setCursor(1, 2);
  lcd.print("Waiting");
  lcd.setCursor(1, 3);
  lcd.print("Next");
}

void menu_along_Z_3() {
  lcd.clear();
  lcd.setCursor(0, cursor_string);
  lcd.write(126);
  lcd.setCursor(1, 0);
  lcd.print("Back");
  lcd.setCursor(1, 1);
  lcd.print("Z Height microstep");
  lcd.setCursor(1, 2);
  lcd.print("Z Step microstep");
  lcd.setCursor(1, 3);
  lcd.print("Start");
}

void menu_along_XZ_1() {
  lcd.clear();
  lcd.setCursor(0, cursor_string);
  lcd.write(126);
  lcd.setCursor(1, 0);
  lcd.print("Back");
  lcd.setCursor(1, 1);
  lcd.print("Height microstep");
  lcd.setCursor(1, 2);
  lcd.print("Step microstep");
  lcd.setCursor(1, 3);
  lcd.print("Next");
}

void menu_along_XZ_2() {
  lcd.clear();
  lcd.setCursor(0, cursor_string);
  lcd.write(126);
  lcd.setCursor(1, 0);
  lcd.print("Back");
  lcd.setCursor(1, 1);
  lcd.print("X 10 mm");
  lcd.setCursor(1, 2);
  lcd.print("X 1 mm");
  lcd.setCursor(1, 3);
  lcd.print("Next");
}

void menu_along_XZ_3() {
  lcd.clear();
  lcd.setCursor(0, cursor_string);
  lcd.write(126);
  lcd.setCursor(1, 0);
  lcd.print("Back");
  lcd.setCursor(1, 1);
  lcd.print("X Step");
  lcd.setCursor(1, 2);
  lcd.print("Start");
}

void select_angle() {
  if (corner < 0) corner = 0;
  if (corner > 180.0) corner = 180.0;
  counter = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Angle");
  lcd.setCursor(0, 1);
  lcd.print(corner);
  lcd.setCursor(6, 1);
  lcd.write(223);
}

void select_step_diameter() {
  if (step_rad < 0) step_rad = 0;
  if (step_rad > 100.0) step_rad = 100.0;
  counter = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Step diameter");
  lcd.setCursor(0, 1);
  lcd.print(step_rad * 2);
  lcd.setCursor(8, 1);
  lcd.print("mm");
}

void select_diameter() {
  if (diameter < 0) diameter = 0;
  if (diameter > 500.0) diameter = 500.0;
  counter = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Diameter:");
  lcd.setCursor(0, 1);
  lcd.print(diameter);
  lcd.setCursor(8, 1);
  lcd.print("mm");
}

void select_x_measuring() {
  if (x_measuring < 0) x_measuring = 0;
  if (x_measuring > 500.0) x_measuring = 500.0;
  counter = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Measuring X:");
  lcd.setCursor(0, 1);
  lcd.print(x_measuring);
  lcd.setCursor(8, 1);
  lcd.print("mm");
}

void select_x_step() {
  x_step += counter * 0.1;
  if (x_step < 0) x_step = 0;
  if (x_step > x_measuring) x_step = x_measuring;
  counter = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Step X:");
  lcd.setCursor(0, 1);
  lcd.print(x_step);
  lcd.setCursor(8, 1);
  lcd.print("mm");
}

void select_z_measuring() {
  if (z_measuring < 0) z_measuring = 0;
  if (z_measuring > 500.0) z_measuring = 500.0;
  counter = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Measuring Z:");
  lcd.setCursor(0, 1);
  lcd.print(z_measuring);
  lcd.setCursor(8, 1);
  lcd.print("mm");
}

void select_z_step() {
  z_step += counter * 0.1;
  if (z_step < 0) z_step = 0;
  if (z_step > z_measuring) z_step = z_measuring;
  counter = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Step Z:");
  lcd.setCursor(0, 1);
  lcd.print(z_step);
  lcd.setCursor(8, 1);
  lcd.print("mm");
}

void select_waiting() {
  waiting += counter * 100;
  if (waiting <= 0) waiting = 0;
  counter = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Waiting:");
  lcd.setCursor(0, 1);
  lcd.print(float(waiting) / 1000.0);
  lcd.setCursor(8, 1);
  lcd.print("s");
}

void select_z_height_microstep() {
  z_height_microstep += counter * 0.1;
  if (z_height_microstep < 0) z_height_microstep = 0;
  if (z_height_microstep > (z_measuring / 2)) z_height_microstep = (z_measuring / 2);
  counter = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Height microstep:");
  lcd.setCursor(0, 1);
  lcd.print(z_height_microstep);
  lcd.setCursor(8, 1);
  lcd.print("mm");
}

void select_z_step_microstep() {
  z_step_microstep += counter * 0.1;
  if (z_step_microstep < 0) z_step_microstep = 0;
  if (z_step_microstep > z_height_microstep) z_step_microstep = z_height_microstep;
  counter = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Step microstep:");
  lcd.setCursor(0, 1);
  lcd.print(z_step_microstep);
  lcd.setCursor(8, 1);
  lcd.print("mm");
}

void cursor() {
  cursor_string += counter;
  counter = 0;
}

void encIsr() {
  enc1.tick();                    // отработка в прерывании
  if (enc1.isRight()) counter++;  // если был поворот
  if (enc1.isLeft()) counter--;
}

void isr() {
  if (millis() - button_timer > 300) {
    button_timer = millis();
    button_flag = 1;
  }
}

void write_to_card() {
  if (card_flag == 0) {
    File dataFile = SD.open("results.csv", FILE_WRITE);
    if (dataFile) {
      if (mode == 4) {
        dataFile.println("Along axis Z");
        dataFile.println("Pressure ; X ; Z");
        dataFile.close();
      }
      if (mode == 5) {
        dataFile.println("Along plane XZ");
        dataFile.println("Pressure ; X ; Z");
        dataFile.close();
      }
      if (mode == 6) {
        dataFile.println("Along circle XZ");
        dataFile.println("Pressure ; X ; Z");
        dataFile.close();
      }
    }
    card_flag = 1;
  }
  results = " ";
  results += PS1.getPressure();
  results += ";";
  results += x_position;
  results += ";";
  results += z_position;
  // Открываем файл, но помним, что одновременно можно работать только с одним файлом.
  // Если файла с таким именем не будет, ардуино создаст его.
  File dataFile = SD.open("results.csv", FILE_WRITE);
  // Если все хорошо, то записываем строку:
  if (dataFile) {
    dataFile.println(results);
    dataFile.close();
  } else {
    // Сообщаем об ошибке, если все плохо
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("error SD");
  }
}
