# ASIMROS's bum-and-go

## Introducción 
El proyecto consta de cuatro ejercicios diferentes:

0. **Basic**: El Kobuki avanza hasta que el bumper detecte un objeto, una vez detectado, retroce y gira hacia la izquierda.
1. **Advanced**: El Kobuki avanza hasta que el bumper detecte un objeto, una vez detectado, retroce y gira al lado contrario al que se ha detectado el objeto. 
2. **Pro**: El Kobuki sigue la misma dinámica que *Advanced*, pero en este caso hace uso del láser para detectar el objeto.

Las velocidades angular y lineal a la que se mueve el Kobuki son pasadas por parámetro.

## BasicBump
Implementa una máquina con tres estados:

##### GOING_FORWARD 
El Kobuki comienza en este estado, avanza hacia delante hasta que detecta con uno de sus bumpers detecta un obstáculo realizando los siguientes cambios:
- Activa la variable *pressed_* a *true*.
- Cambia el estado, *state_* pasa a ser *GOING_BACK*. 
- Guarada el tiempo en el que se presionó un bumper en la variable *press_ts_*.

##### GOING_BACK
El Kobuki retroce mientras la diferencia entre el tiempo actual y el tiempo almacenado en *turn_ts_* es inferior a la constante *BACKING_TIME*. Una vez superado, el estado cambia a *TURNING* y se apunta en la variable *turn_ts_* el tiempo en el que empieza a girar.

##### TURNING 
El Kobuki gira a la izquierda mientras la diferencia entre el tiempo actual y el tiempo almacenado en *turn_ts_* es inferior a *TURNING_TIME*. A continuación, vuelve al estado *GOING_FORWARD*.

## Base
Tanto ***Advanced*** como ***ProBumper*** heredan de la clase *Base* la mayoría de sus variables y la máquina de estados, teniendo en este caso cuatro estados:
##### GOING_FORWARD 
Mientras se mantenga yendo hacia delante comprueba si se ha detectado un obstáculo , en caso de que no haya ocurrido sigue avanzando a una velocidad pasada por parámetros. Si por el contrario si ha detectado un obstáculo apunta el momento en que se ha dado cuenta y cambia su estado a *GOING_BACK*. 

##### GOING_BACK
Retrocede a la misma velocidad hasta que el tiempo desde que se detectó un obstáculo es superior al especificado en la constante "BACKING_TIME". Una vez agotado este tiempo apunta el momento en que deja de retroceder y comprueba la variable ***side_***, actualizada por los callbacks de ***Advanced** y ***ProBumper***. Si ha detectado el objeto en el lado izquierdo pasa al estado "TURNING_RIGHT", en cualquier otro caso pasa a "TURNING_LEFT".

##### TURNING_LEFT
Para el movimiento linear y gira en sentido antihorario a una velocidad pasada hasta que el tiempo desde que pasó desde que dejó de retroceder supera el valor de la constante "TURNING_TIME". A continuación vuelve al estado "GOING_FORWARD".

##### TURNING_RIGHT 
Para el movimiento linear y gira en sentido horario a una velocidad pasada hasta que el tiempo desde que pasó desde que dejó de retroceder supera el valor de la constante "TURNING_TIME". A continuación vuelve al estado "GOING_FORWARD".

### Advanced
Hereda todas sus variables de ***Base*** y únicamente comprueba en su callback cuando ha entrado en contacto con un obstáculo y cuál de sus *bumpers* ha sido el que se ha presionado para actualizar la variable ***side_***.

## ProBumper
Al igual que en el ejercicio anterior, *advanced*, hereda la máquina de estados diseñada en la clase padre *base.h*, la cuál consta de 4 estados:

#### GOING FORWARD

Realiza la misma acción que en el *advanced*, con la diferencia de que el obstáculo se detecta utilizando el láser. Cuando dicho objeto se encuentre delante del kobuki, a una distancia inferior a la pasada por parámetro, se activa la variable *detected_* a true, cambiando al estado *GOING_BACK*.

Este proceso de detectar el obstáculo, se realiza en la función *detectInRange(msg)*, la cual devuelve un entero con la posición del array en la que se encuentra dicho obstáculo, guardándola en *index_*.

El rango en el que tiene que analizar si hay un obstáculo o no, se ha calculado con la arcotangente de la división del radio entre la distancia a la que queremos detectarlo: arctg(radio/distancia). Este número sumado al angle_min y restado al angle_max, nos da el rango de medición. 

Una vez se ha guardado *index_*, se comprueba si el índice pertenece a la primera parte del array (se encuentra a la izquierda), o a la parte final (situada a la derecha), guardando en la variable *side_ RIGHT* o *LEFT*, la dirección por la que esquiva el obstáculo.

#### GOING_BACK

Se realiza igual que en el ejercicio *advanced*, el kobuki retrocede durante un tiempo determinado, y posteriormente cambia de estado a *TURNING_LEFT* o *TURNING_RIGHT*, dependiendo del valor de *side_*.

#### TURNING_RIGHT Y TURNING_LEFT

El kobuki gira hacia el lado correspondiente durante un tiempo determinado, de la misma forma que en advanced


## Members

* Juan Simó
* Ruben Montes
* Lara Poves
* Óscar Martínez
