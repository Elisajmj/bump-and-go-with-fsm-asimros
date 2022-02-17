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

### ProBumper
A parte de las variables heredadas contiene variables para saber a qué distancia frenar y qué ángulos tener en cuenta para decidir el lado hacia el que girar.
Además del callback tiene una función "detectInRange" que determina si hay un obstáculo y el "ángulo" en que lo detecta. Además, evita que se comprueben los datos del láser de detrás del robot.

El callback, si el estado es "GOING_FORWARD", llama a "detectInRange" y actualiza la variable size según el ángulo, también pone ***detected_ a true***.
En caso de que no encuentre nada a la distancia configurada pone ***detected_*** a false.
Si no está yendo hacia alante se salta todo eso, ya que el láser sigue mandando información mientras retrocede o gira, y así evita que se procesen datos inecesarios.

## MegaPro
El objetivo de este programa es que escoga la dirección hacia la que ir,en su callback si va hacia alante comprueba si hay obstáculos igual que el ***ProBumper***, por otro lado, si está leyendo decide el mejor camino con la función "detectBetterOption" y actualiza la variable ***indexfar_*** para saber hacia que ángulo ir.
Tiene solo tres estados:

##### GOING_FORWARD
Que funciona igual que en el caso del ***ProBumper***.

##### READING
Apunta el momento en que ha dejado de avanzar ...

##### TURNING

## Members

* Juan Simó
* Ruben Montes
* Lara Poves
* Óscar Martínez
