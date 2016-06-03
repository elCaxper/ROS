/*
 * gusano.h
 *
 *  Created on: 20 de feb. de 2016
 *      Author: kaiser
 */

#ifndef Worm_h
#define Worm_h

#include "OscillatorSerp.h"

//-- Maximo numero de servos
#define MAX_SERVOS 9

//-- Estructura de los parametros de la ola
struct wave_s {
  unsigned int T;  //-- periodo (ms)
  int A;  //-- amplitud (deg)
  int O;           //-- offset (deg)
  int PD;          //-- diferencia de fase (deg)
  int PHASE0;      //-- fase inicial (deg)
};

typedef struct wave_s Wave;

class Gusano
{
  public:

    //-- Constructor por defecto
    Gusano();

    //-- Añadir un nuevo servo
    void add_servo(std::string topico);

    //-- asignar una ola al gusano
    void set_wave(Wave w, int servo = -1);

    //-- ajustar amplitud
    void SetA(int A, int servo  = -1);

    //-- ajustar periodo (ms)
    void SetT(unsigned int T, int servo  = -1);

    //-- ajustar offset (deg)
    void SetO(int O, int servo  = -1);

    //-- ajustar diferencia de fase (deg)
    void SetPd(int Pd, int servo = -1);

    //-- ajstar fase inicial (deg)
    void SetPh0(int Ph0);

    //-- actualizar el estado del gusano
    void refresh();


  private:
    //-- numero de servos del gusano
    int _nservos;

    //-- osciladores
    OscillatorSerp Osc[MAX_SERVOS];

    //-- fase inicial
    int _phase0;

    //-- diferencia de fase inicial
    int _diferencia_fase;
};

#endif
