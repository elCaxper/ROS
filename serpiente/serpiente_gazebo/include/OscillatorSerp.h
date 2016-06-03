/*
 * OscillatorSerpSerp.h
 *
 *  Created on: 20 de feb. de 2016
 *      Author: kaiser
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "stdlib.h"

#ifndef OscillatorSerp_h
#define OscillatorSerp_h


//-- Macro para pasar grados a radianes
#ifndef DEG2RAD
  #define DEG2RAD(g) ((g)*M_PI)/180
#endif

class OscillatorSerp
{
  public:
    OscillatorSerp(){};
    void attach(std::string topico);
    void detach();

    void SetPosicion(int A) {_Amplitud_movimiento=A;};
    void SetOffset(unsigned int O) {_Offset_articulacion=O;};
    void SetFase(double Ph) {angulo_actual_inicial=Ph;};
    void SetPeriodo(unsigned int T);
    void SetPosition(int position);
    void StopServo() {_pararServo=true;};
    void PlayServo() {_pararServo=false;};
    void ResetServo() {angulo_actual=0;};
    void actualizaPosicionServo();

  private:
    bool next_sample();

  private:
    //-- Manejador del nodo
    ros::NodeHandle n;
    //-- Publicador
    ros::Publisher _servo ;
    //-- Dato a publicar
    std_msgs::Float64 datoArticulacion;

    //-- Paramentros del servo
    unsigned int _Amplitud_movimiento;  //-- Amplitud (grados)
    unsigned int _Offset_articulacion;  //-- Offset (grados)
    unsigned int _Periodo_actualizacion;  //-- Periodo (milisegundos)
    double angulo_actual_inicial;   //-- Fase (rad)

    //-- Internal variables
    int _pos_actual;         //-- Posicion actual
    double angulo_actual;    //-- Fase Actual
    double _incremento_fase;      //-- Incremento de fase
    double _Numero_muestras;        //-- Numero de muestras
    unsigned int _Periodo_muestreo; //-- Periodo de muestreo (ms)

    long _previousMillis;
    long _currentMillis;

    //-- Variable para parar el servo
    bool _pararServo;

    //-- Para controlar el servo en sentido contrario
    bool _sentidoContrario;
};

#endif
