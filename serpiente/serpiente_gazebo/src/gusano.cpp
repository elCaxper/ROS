/*
 * gusano.cpp
 *
 *  Created on: 20 de feb. de 2016
 *      Author: kaiser
 */

#include "gusano.h"

#include "OscillatorSerp.h"

//-- Macro for converting from degrees to radians
#define DEG2RAD(g) ((g)*M_PI)/180

Gusano::Gusano()
{
  //-- Initially the robot consist of 0 servos
  _nservos=0;

  //-- Initial phase
  _phase0=0;

  //-- Initial phase difference
  _diferencia_fase=-120;
}

void Gusano::add_servo(std::string topico)
{
   Osc[_nservos].attach(topico);

   //-- Calculate the default phase
   Osc[_nservos].SetFase(DEG2RAD(_phase0 + _nservos*_diferencia_fase));
   _nservos++;
}

void Gusano::refresh()
{
  //-- Refresh the oscillators
  for (int i=0; i<_nservos; i++)
    Osc[i].actualizaPosicionServo();
}


void Gusano::set_wave(Wave w, int servo )
{
	//-- Asignamos la ola a un servo o a todos
	if (servo >= 0 && servo < _nservos)
	{
   		Osc[servo].SetPosicion(w.A);
    	Osc[servo].SetOffset(w.O);
		Osc[servo].SetPeriodo(w.T);
		Osc[servo].SetFase(DEG2RAD(w.PD));
	}
	else
	{
 		for (int i=0; i<_nservos; i++)
		{
   			Osc[i].SetPosicion(w.A);
   			Osc[i].SetOffset(w.O);
			Osc[i].SetPeriodo(w.T);
			Osc[i].SetFase(DEG2RAD(w.PHASE0 + i*w.PD));
		}
	}

  _phase0=w.PHASE0;
}

void Gusano::SetA(int A,  int servo)
{
	//-- If a servo index is specified, it only changes that servo
	if (servo >= 0 && servo < _nservos)
		Osc[servo].SetPosicion(A);
	else
  		for (int i=0; i<_nservos; i++)
   			Osc[i].SetPosicion(A);
}

void Gusano::SetT(unsigned int T,  int servo)
{
	//-- If a servo index is specified, it only changes that servo
	if (servo >= 0 && servo < _nservos)
		Osc[servo].SetPeriodo(T);
	else
  		for (int i=0; i<_nservos; i++)
    			Osc[i].SetPeriodo(T);
}

void Gusano::SetO(int O,  int servo)
{
	//-- If a servo index is specified, it only changes that servo
	if (servo >= 0 && servo < _nservos)
		Osc[servo].SetOffset(O);
	else
		for (int i=0; i<_nservos; i++)
		    Osc[i].SetOffset(O);
}

void Gusano::SetPd(int Pd, int servo)
{
	//-- If a servo index is specified, it only changes that servo
	if (servo >= 0 && servo < _nservos)
		Osc[servo].SetFase(DEG2RAD(Pd));
	else
		for (int i=0; i<_nservos; i++)
		Osc[i].SetFase(DEG2RAD(_phase0 + i*Pd));
}

void Gusano::SetPh0(int Ph0)
{
  _phase0 = Ph0;
  SetPd(_diferencia_fase);
}
