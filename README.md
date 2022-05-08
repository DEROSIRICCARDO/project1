# project1
Student ID: 10602888		name: Riccardo		Surname: De Rosi
Student ID: 10661470		name: Marco		Surname: Corso
Student ID: 		name: Luca		Surname: Lobriglio

Abbiamo creato diverse cartelle tra cui bags, cfg, include, launch, msg, parameters, rviz, src, srv e dei file tra cui CMakelists. 
La cartella bags contiene i 3 file (bag) con i messaggi degli encoder. 
Cfg contiene il file in cui viene descritta la funzione dynamic.reconfigure per poter selezionare in metodo di integrazione (Eulero o Runge Kutta).
Include è diviso tra l'odometria e la velocità ma in generale, in questi file, vengono richiamate le varie funzioni e le variabili.
Launch contiene il file .launch che fa partire i vari nodi, parametri, il programma rviz.
Msg contiene 2 file nei quali viene descritto la struttura del messaggio.
Parameters contiene i valori dei vari parametri utilizzati nei vari nodi (esempio la frequenza e i valori dati nel testo).
Rviz contiene il file per aprire il programma rviz con già preimpostati tutti i valori.
Src contiene tutti i file dei nodi. Il Client serve per resettare la posizione e sincronizzarla con quella della bag. In inv_velocity e velocity vengono descritte la varie funzione del nodo per calcolare la cinematica diretta e inversa. Nell'odometer invece vengono descritte le funzione per calcolare l'odometria.
Srv contiene il file per il reset (nello specifico descrive la struttura della posizione)

I parametri che abbiamo aggiunto sono il looprate che è la frequenza con la quale si aggiorna il nodo, i 5 valori dati dal problema (l, w, r, T, N) e il valore iniziale della posizione lungo x, y e dell'angolo theta.

La struttura è formata da world map odom robot.

Abbiamo creato 2 tipi di messaggi chiamati ParamMsm e Wrpm. Il primo è usato per la calibrazione ed è un vettore formato dalle tre variabili lw(l+w), r e N. Il secondo è sempre un vettore formato dalla velocità di ogni ruota e viene usato nella cinematica inversa.

Per far partire i nodi basta far partire il file .launch.
