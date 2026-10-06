# PRACTICA 1
## Navegación de un robot aspiradora de gama baja
Lo primero que he realizado para poder realizar esta practica es mirar que _import_ son necesarios para poder usar esas librerias para que el robot funcione. Una vez visto los _import_ defino el tipo de algoritmo que me gustaria usar para realizar esta practica. En este caso se nos da como algoritmo obligatorio el algoritmo de **espiral**, el cual he usado pero aplicandole una pequeña modificación ya que no aplica el comando de espiral constantemente.\
\
Pero antes de aplicar el algortimo al codigo y probarlo, defino la tabla de estados que voy a tener en el codigo. En mi caso hago una tabla con 4 estados:\
    - **SPIRAL** (Este es un añadido ya que es el algoritmo y en el enunciado se nos dice que nuestro codigo tiene que tener al menos 3 estados, que son los siguentes que menciono)\
    - **DASH**\
    - **TURN**\
    - **BACK**\
\
En mi caso he planteado que empieze con el estado de espiral, pero despues de chocarse o encontrar un objeto simplente avanza. Pero planteo dos situaciones por tiempo:\
    - En menos de 8 segundos: Si antes de 8 segundos el robot encuentra otro objeto, el tiempo se reinicia y vuelve a aplicar el estado de volver hacia atras, girar sobre si mismo y finalmente avanzar hacia delate.\
    - Justo en 8 segundos: En caso de que el tiempo llegue a 8 segundos sin encontrar un obstaculo, procede ha iniciar de nuevo ese estado de espiral.\
\
Una vez ya tengo la estructura del código paso a crearlo, dandole forma con funciones y dando unos valores genericos (ya que todavia no hemos procedido ha hacer una prueba experimental y no sabemos que valores asignarle a cada cosa para que el funcionamiento del robot sea el más óptimo).
## Explicación de funciones
`elapsed()`
\
Esta función cuenta el tiempo que lleva el robot en ese estado, dicho de otra forma es un cronometro.\
`obstacle()`
\
Esta función hace uso de los laser del robot para la detección de objetos. La funcion devuelve TRUE o FALSE segun lo que detecten los rayos. En caso de que el rayo no detecte nada devuelve FALSE, y en caso de que el rayo detecte un objeto y ese objeto este dentro de la distancia definida devuelve TRUE.
\
`on_enter()`
\
Esta función se ejecuta una _unica vez_ cada vez que se realiza un cambio de estado. Dento de la función se realiza el cambio de estado, reinicia el cornometro para la funcion `elapsed()` y en caso de que el estado al que cambie sea **TURN** valora el tiempo de giro y la dirección de giro. Implementando asi la aleatoriedad para no repetir el mismo giro y por ende entrar en un bucle pasando por los mismo sitios.

## Ejecución
Finalmente el código ejecuta un bucle infinito, en el que ejecuta cada estado de la tabla de estados y en el que se ejecuta la función `on_enter()` para poder realizar el cambio de estados. Dentro del bucle tengo una funcion llamada `Frequency.tick()` que lo que hace es que mantenga el bucel en ejecución a 20Hz, o dicho de otra forma que se realizen 20 iteraciones por minuto.
## Video
## Imagen
Durante varias pruebas cambiando tanto el codigo como los datos experimentales para la ejecución, en el caso de la imagen el código en uso es el presentado en la entrega, vemos que el robot ha logrado realizar la limpieza al 101% en aproximadamente 1 hora y 22 minutos.
\
![Resultado de prueba](P1/mapacompleto.jpg)