# project1
First robotics project 2022
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

Nell'odometria i parametri di ros che richiamiamo sono i valori della posizione lungo x e lungo y e il valore dell'angolo theta. Nella cinematica richiamiamo solo il valore della frequenza (LoopRate).


Per far partire i nodi basta far partire il file .launch.
