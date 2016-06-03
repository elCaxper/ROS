/*
 * OscillatorSerpSerp.cpp
 *
 *  Created on: 20 de feb. de 2016
 *      Author: kaiser
 */

#include "OscillatorSerp.h"
#include "time.h"
/*#ifndef M_PI
#define M_PI 3.141592
#endif*/


bool OscillatorSerp::next_sample()
{

	// Tiempo funcionando
	_currentMillis =  (double)(clock())/CLOCKS_PER_SEC*1000;

	// Vemos si hay que actualizar
	if(_currentMillis - _previousMillis > _Periodo_muestreo) {
		_previousMillis = _currentMillis;

		return true;
	}

	return false;
}

//-- Asignamos al servo el tópico y valores por defecto
void OscillatorSerp::attach(std::string topico)
{
	_servo = n.advertise<std_msgs::Float64>(topico, 1000);

	datoArticulacion.data=0;
	_servo.publish(datoArticulacion);

	_Periodo_muestreo=30;
	_Periodo_actualizacion=2000;
	_Numero_muestras = _Periodo_actualizacion/_Periodo_muestreo;
	_incremento_fase = 2*M_PI/_Numero_muestras;

	_previousMillis=0;

	//-- Parametros por Defecto
	_Amplitud_movimiento=45;
	angulo_actual=0;
	angulo_actual_inicial=0;
	_Offset_articulacion=0;
	_pararServo=false;

	//-- Cambio de sentido Servo
	_sentidoContrario = false;

}

//-- Detach an OscillatorSerp from his servo

/*************************************/
/* Set the OscillatorSerp period, in ms  */
/*************************************/
void OscillatorSerp::SetPeriodo(unsigned int T)
{
	//-- Asignamos el nuevo valor
	_Periodo_actualizacion=T;

	//-- Recalculamos los parametros
	_Numero_muestras = _Periodo_actualizacion/_Periodo_muestreo;
	_incremento_fase = 2*M_PI/_Numero_muestras;
};

//-- Establecer posicion
void OscillatorSerp::SetPosition(int position)
{
	datoArticulacion.data=position;
	_servo.publish(datoArticulacion);
};


//-- Llamamos periodicamente para actualizar la posicion del servo
void OscillatorSerp::actualizaPosicionServo()
{
	//-- Si hay que actualizar
	if (next_sample()) {

		//-- Si no esta parado
		if (!_pararServo) {
			//-- Posicion servo
			_pos_actual = round(_Amplitud_movimiento * sin(angulo_actual + angulo_actual_inicial + _Offset_articulacion));
			if (_sentidoContrario) _pos_actual=-_pos_actual;
			datoArticulacion.data=_pos_actual+90;
			datoArticulacion.data-=90;
			datoArticulacion.data=DEG2RAD(datoArticulacion.data);
			_servo.publish(datoArticulacion);
		}

		//-- Incrementamos la fase
		angulo_actual = angulo_actual + _incremento_fase;

	}
}
