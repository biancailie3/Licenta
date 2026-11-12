#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <stdint.h>
#include <pthread.h>
#include <sched.h>
#include <lgpio.h>

/* ===================== 1. PINI (numerotare GPIO/BCM) ===================== */

/* Motorul DREAPTA (driver L298N, canalul A) */
#define PIN_MOTOR_DREAPTA_VITEZA_PWM     12   
#define PIN_MOTOR_DREAPTA_DIRECTIE_1      5   
#define PIN_MOTOR_DREAPTA_DIRECTIE_2      6  

/* Motorul STANGA (driver L298N, canalul B) */
#define PIN_MOTOR_STANGA_VITEZA_PWM      13   
#define PIN_MOTOR_STANGA_DIRECTIE_1      19   
#define PIN_MOTOR_STANGA_DIRECTIE_2      26   

/* Servomotoare */
#define PIN_SERVO_ORIZONTAL              18   
#define PIN_SERVO_VERTICAL               25   

/* Senzor ultrasonic HC-SR04 */
#define PIN_SENZOR_TRIGGER               23   
#define PIN_SENZOR_ECHO                  24  

#define HARTA_3D_ACTIVA 0                    

/* ===================== PARAMETRI DE SCANARE ===================== */

#define UNGHI_SERVO_MINIM_GRADE          10   
#define UNGHI_SERVO_MAXIM_GRADE         100   
#define PAS_UNGHI_SERVO_GRADE            10   
#define NUMAR_UNGHIURI_MASURATE  ((UNGHI_SERVO_MAXIM_GRADE - UNGHI_SERVO_MINIM_GRADE) / PAS_UNGHI_SERVO_GRADE + 1) 
#define NUMAR_PASI_PE_TRECERE    ((UNGHI_SERVO_MAXIM_GRADE - UNGHI_SERVO_MINIM_GRADE) / PAS_UNGHI_SERVO_GRADE)     
#define DURATA_TRECERE_COMPLETA_S       0.85   
#define SENZOR_PRIVESTE_STANGA_LA_UNGHI_MARE 0 //priveste spre dreapta
#define SERVO_IMPULS_MINIM_US           500  
#define SERVO_IMPULS_MAXIM_US          2500   
#define INALTIME_SENZOR_DE_LA_PODEA_CM  12.0  
#define DISTANTA_FARA_OBSTACOL_CM      999.0  
#define VARSTA_MAXIMA_MASURATOARE_S      2.5  

/* ===================== 2. GEOMETRIE / DISTANTE (cm) ===================== */

#define SEMILATIME_ROBOT_CU_MARJA_CM            10.0  
#define DEPASIRE_SENZOR_FATA_DE_SASIU_CM         0.0  
#define DISTANTA_OPRIRE_FATA_DE_OBSTACOL_CM      6.0  
#define DISTANTA_PARCURSA_IN_TIMPUL_FRANARII_CM  1.5  
#define DISTANTA_INCEPUT_INCETINIRE_CM          40.0  
#define DISTANTA_DORITA_FATA_DE_PERETE_STANG_CM 25.0 
#define DISTANTA_PERETE_STANG_PIERDUT_CM        70.0  
#define DISTANTA_RETRAGERE_CM                    6.0  
#define UNGHI_ALINIERE_LA_PERETE_GRADE          85.0  

/* ===================== COMENZI SI MOTOARE (0..1) ===================== */

#define VITEZA_CROAZIERA                 0.22   
#define VITEZA_MINIMA_APROPIERE          0.12   
#define VITEZA_ROTIRE_PE_LOC             0.80   
#define FRECVENTA_PWM_HZ               500.0    
#define PWM_MINIM_PORNIRE_MOTOR          0.38   
#define PWM_MAXIM_MOTOR                  0.80   
#define PWM_IMPULS_PORNIRE               0.90  
#define DURATA_IMPULS_PORNIRE_S          0.12   
#define DURATA_FRANARE_ACTIVA_S          0.12 
#define CRESTERE_MAXIMA_COMANDA_PE_SECUNDA 3.0  
#define TIMEOUT_COMANDA_WATCHDOG_S       0.40   

/* ===================== ODOMETRIE (DE CALIBRAT cu ./robot calib-mers si calib-viraj) ===================== */

#define VITEZA_LINIARA_CM_S_LA_COMANDA_MAXIMA    121.0  
#define VITEZA_UNGHIULARA_GRADE_S_LA_COMANDA_MAXIMA 192.0 

/* ===================== CONTROL URMARIRE PERETE ===================== */

#define CASTIG_PROPORTIONAL_PERETE       1.5    
#define CASTIG_EVITARE_OBSTACOL_FATA     1.5    
#define CASTIG_CURBARE_ARC               1.0    
#define VIRAJ_CAUTARE_PERETE_PIERDUT     0.6   
#define PAS_ROTIRE_CAND_BLOCAT_GRADE    20.0    

/* ===================== DETECTAREA SFARSITULUI SCANARII (inchidere bucla) ===================== */

#define ROTATIE_MINIMA_TURA_COMPLETA_GRADE 330.0  
#define DRUM_MINIM_TURA_CM                 150.0  
#define RAZA_INCHIDERE_BUCLA_CM             40.0  
#define TOLERANTA_PROFIL_SCANARE_CM         25.0  
#define TIMP_MAXIM_TURA_S                  600.0  

#define GRADE_IN_RADIANI (M_PI / 180.0)         

/* ===================== 3. STARE PARTAJATA INTRE THREAD-URI ===================== */

static volatile sig_atomic_t g_programul_ruleaza = 1;   
static int g_handle_gpio = -1;                          

/* Pozitia robotului estimata prin odometrie */
typedef struct {
    double x_cm;                        
    double y_cm;                      
    double orientare_grade;             
    double rotatie_cumulata_grade;      
    double drum_parcurs_cm;             
    double viteza_cm_s;                 
} PozitieRobot;                        
static PozitieRobot g_pozitie_robot;    
static pthread_mutex_t mutex_pozitie = PTHREAD_MUTEX_INITIALIZER;   

/* Ultimele distante masurate la fiecare unghi al servo-ului */
typedef struct {
    double distante_cm[NUMAR_UNGHIURI_MASURATE];        
    double momente_masuratorii_s[NUMAR_UNGHIURI_MASURATE]; 
    double moment_ultima_masuratoare_s;                 
    int numar_treceri;                                 
} DateScanare;                                          
static DateScanare g_scanare;                           
static pthread_mutex_t mutex_scanare = PTHREAD_MUTEX_INITIALIZER;   

/* Comanda pentru motoare, scrisa de navigare si citita de thread-ul motoarelor */
typedef struct {
    double viteza_stanga;               
    double viteza_dreapta;              
    int franare_activa;                 
    double moment_emiterii_s;           
} ComandaMotoare;                       
static ComandaMotoare g_comanda_motoare;   
static pthread_mutex_t mutex_comanda = PTHREAD_MUTEX_INITIALIZER;  

/* Cel mai apropiat perete gasit in timpul rotirii pe loc */
static struct {
    int activ;                          
    double distanta_cm;                 
    double directie_grade;              
} g_perete_cel_mai_apropiat;           
static pthread_mutex_t mutex_perete_apropiat = PTHREAD_MUTEX_INITIALIZER;  

static int g_inclinare_servo_grade = 0; 

/* Coada de puncte pentru harta (producator: SCANARE, consumator: HARTA) */
typedef struct {
    PozitieRobot pozitie_robot;         
    int unghi_servo_grade;             
    int inclinare_grade;                
    double distanta_cm;                 
} PunctHarta;                           
#define CAPACITATE_COADA_PUNCTE 8192  
static PunctHarta coada_puncte_harta[CAPACITATE_COADA_PUNCTE]; 
static int coada_index_inceput = 0;     
static int coada_numar_elemente = 0;    
static int coada_oprire_ceruta = 0;     
static long coada_puncte_pierdute = 0;  
static pthread_mutex_t mutex_coada = PTHREAD_MUTEX_INITIALIZER;    
static pthread_cond_t conditie_coada_nevida = PTHREAD_COND_INITIALIZER;

typedef enum {
    FAZA_ASTEAPTA,          
    FAZA_SCAN_360,         
    FAZA_CAUTA_PERETE,      
    FAZA_APROPIE,           
    FAZA_RETRAGE,           
    FAZA_ALINIEAZA,         
    FAZA_URMARESTE_PERETE, 
    FAZA_GATA              
} FazaNavigare;            
static const char *NUME_FAZE[] = { "ASTEAPTA", "SCAN360", "CAUTA_PERETE", "APROPIE",
                                   "RETRAGE", "ALINIEAZA", "URMARESTE_PERETE", "GATA" };  
static volatile int g_faza_curenta = FAZA_ASTEAPTA;     
static volatile int g_servo_fix_inainte = 0;            

/* ===================== 3. UTILITARE ===================== */

static double timp_curent_s(void) {                   
    struct timespec moment;
    clock_gettime(CLOCK_MONOTONIC, &moment);
    return moment.tv_sec + moment.tv_nsec * 1e-9;
}
static uint64_t timp_curent_ns(void) {                  
    struct timespec moment;
    clock_gettime(CLOCK_MONOTONIC, &moment);
    return (uint64_t)moment.tv_sec * 1000000000ULL + (uint64_t)moment.tv_nsec;
}
static void asteapta_activ_microsecunde(int microsecunde) {    
    uint64_t moment_start_ns = timp_curent_ns();
    while (timp_curent_ns() - moment_start_ns < (uint64_t)microsecunde * 1000ULL) { }
}
static void dormi_secunde(double secunde) {             
    struct timespec durata;
    durata.tv_sec = (time_t)secunde;
    durata.tv_nsec = (long)((secunde - (double)durata.tv_sec) * 1e9);
    nanosleep(&durata, NULL);
}
static double limiteaza_valoare(double valoare, double minim, double maxim) {   
    return valoare < minim ? minim : (valoare > maxim ? maxim : valoare);
}
static double normalizeaza_unghi_180(double unghi_grade) {     
    while (unghi_grade > 180.0) unghi_grade -= 360.0;
    while (unghi_grade < -180.0) unghi_grade += 360.0;
    return unghi_grade;
}
static void seteaza_prioritate_timp_real(int prioritate) {      
    struct sched_param parametri;
    memset(&parametri, 0, sizeof parametri);
    parametri.sched_priority = prioritate;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &parametri) != 0)
        fprintf(stderr, "[info] prioritate real-time indisponibila (ruleaza cu sudo ca s-o primesti)\n");
}
/* unghiul senzorului fata de directia de mers, in grade, + = stanga */
static double unghi_senzor_spre_stanga_grade(int unghi_servo_grade) {  
    return SENZOR_PRIVESTE_STANGA_LA_UNGHI_MARE ? (unghi_servo_grade - 90.0) : (90.0 - unghi_servo_grade);
}
static void la_primire_semnal(int semnal) { (void)semnal; g_programul_ruleaza = 0; }   

static PozitieRobot citeste_pozitie_robot(void) {      
    pthread_mutex_lock(&mutex_pozitie);
    PozitieRobot pozitie = g_pozitie_robot;
    pthread_mutex_unlock(&mutex_pozitie);
    return pozitie;
}
static void copiaza_date_scanare(DateScanare *destinatie) {     
    pthread_mutex_lock(&mutex_scanare);
    *destinatie = g_scanare;
    pthread_mutex_unlock(&mutex_scanare);
}
static void seteaza_comanda_motoare(double viteza_stanga, double viteza_dreapta, int franare_activa) {   
    pthread_mutex_lock(&mutex_comanda);
    g_comanda_motoare.viteza_stanga = viteza_stanga;
    g_comanda_motoare.viteza_dreapta = viteza_dreapta;
    g_comanda_motoare.franare_activa = franare_activa;
    g_comanda_motoare.moment_emiterii_s = timp_curent_s();
    pthread_mutex_unlock(&mutex_comanda);
}
static void comanda_franare(void) { seteaza_comanda_motoare(0.0, 0.0, 1); }    

/* ===================== 4. HARDWARE (lgpio) ===================== */

/* Index 0 = motor stanga, index 1 = motor dreapta */
static const int PINI_VITEZA_PWM_MOTOR[2]   = { PIN_MOTOR_STANGA_VITEZA_PWM,   PIN_MOTOR_DREAPTA_VITEZA_PWM };   
static const int PINI_DIRECTIE_1_MOTOR[2]   = { PIN_MOTOR_STANGA_DIRECTIE_1,   PIN_MOTOR_DREAPTA_DIRECTIE_1 };   
static const int PINI_DIRECTIE_2_MOTOR[2]   = { PIN_MOTOR_STANGA_DIRECTIE_2,   PIN_MOTOR_DREAPTA_DIRECTIE_2 };   

static int initializeaza_hardware(void) {             
    g_handle_gpio = lgGpiochipOpen(0);
    if (g_handle_gpio < 0) { fprintf(stderr, "Nu pot deschide gpiochip0 (cod %d)\n", g_handle_gpio); return 0; }
    int pini_iesire[] = { PIN_MOTOR_DREAPTA_VITEZA_PWM, PIN_MOTOR_DREAPTA_DIRECTIE_1, PIN_MOTOR_DREAPTA_DIRECTIE_2,
                          PIN_MOTOR_STANGA_VITEZA_PWM, PIN_MOTOR_STANGA_DIRECTIE_1, PIN_MOTOR_STANGA_DIRECTIE_2,
                          PIN_SENZOR_TRIGGER, PIN_SERVO_ORIZONTAL
#if HARTA_3D_ACTIVA
                  , PIN_SERVO_VERTICAL
#endif
    };
    for (unsigned i = 0; i < sizeof pini_iesire / sizeof pini_iesire[0]; i++)
        lgGpioClaimOutput(g_handle_gpio, 0, pini_iesire[i], 0);
    lgGpioClaimInput(g_handle_gpio, 0, PIN_SENZOR_ECHO);
    return 1;
}

/* directie: 1 inainte, -1 inapoi, 0 liber (coast), 2 franare */
static void aplica_comanda_pe_motor(int index_motor, int directie, double factor_umplere) {     
    switch (directie) {
    case 1:  lgGpioWrite(g_handle_gpio, PINI_DIRECTIE_1_MOTOR[index_motor], 0); lgGpioWrite(g_handle_gpio, PINI_DIRECTIE_2_MOTOR[index_motor], 1); break;
    case -1: lgGpioWrite(g_handle_gpio, PINI_DIRECTIE_1_MOTOR[index_motor], 1); lgGpioWrite(g_handle_gpio, PINI_DIRECTIE_2_MOTOR[index_motor], 0); break;
    case 2:  lgGpioWrite(g_handle_gpio, PINI_DIRECTIE_1_MOTOR[index_motor], 1); lgGpioWrite(g_handle_gpio, PINI_DIRECTIE_2_MOTOR[index_motor], 1); factor_umplere = 1.0; break;
    default: lgGpioWrite(g_handle_gpio, PINI_DIRECTIE_1_MOTOR[index_motor], 0); lgGpioWrite(g_handle_gpio, PINI_DIRECTIE_2_MOTOR[index_motor], 0); factor_umplere = 0.0; break;
    }
    lgTxPwm(g_handle_gpio, PINI_VITEZA_PWM_MOTOR[index_motor], (float)FRECVENTA_PWM_HZ, (float)(factor_umplere * 100.0), 0, 0);
}

static void pozitioneaza_servo(int pin_servo, double unghi_grade) {     
    int latime_impuls_us = SERVO_IMPULS_MINIM_US + (int)(unghi_grade * (SERVO_IMPULS_MAXIM_US - SERVO_IMPULS_MINIM_US) / 180.0);
    lgTxServo(g_handle_gpio, pin_servo, latime_impuls_us, 50, 0, 0);
}
static void opreste_servo(int pin_servo) { lgTxServo(g_handle_gpio, pin_servo, 0, 50, 0, 0); }  

/* HC-SR04: timpul ecoului masurat prin polling cu ceas monoton.
 * Intoarce distanta in cm sau -1 daca nu exista ecou valid. */
static double masoara_distanta_ultrasonic_cm(void) {    
    lgGpioWrite(g_handle_gpio, PIN_SENZOR_TRIGGER, 0);
    asteapta_activ_microsecunde(2);
    lgGpioWrite(g_handle_gpio, PIN_SENZOR_TRIGGER, 1);
    asteapta_activ_microsecunde(12);
    lgGpioWrite(g_handle_gpio, PIN_SENZOR_TRIGGER, 0);

    uint64_t moment_asteptare_ecou_ns = timp_curent_ns();
    while (lgGpioRead(g_handle_gpio, PIN_SENZOR_ECHO) == 0) {
        if (timp_curent_ns() - moment_asteptare_ecou_ns > 8000000ULL) return -1.0;     /* nu a pornit ecoul */
    }
    uint64_t moment_inceput_ecou_ns = timp_curent_ns();
    while (lgGpioRead(g_handle_gpio, PIN_SENZOR_ECHO) == 1) {
        if (timp_curent_ns() - moment_inceput_ecou_ns > 25000000ULL) return -1.0;      /* peste ~4 m */
    }
    uint64_t moment_sfarsit_ecou_ns = timp_curent_ns();
    double distanta_cm = (double)(moment_sfarsit_ecou_ns - moment_inceput_ecou_ns) / 1000.0 * 0.01715;   /* 343 m/s, dus-intors */
    if (distanta_cm < 2.0 || distanta_cm > 400.0) return -1.0;
    return distanta_cm;
}

/* ===================== 5. COADA PENTRU HARTA ===================== */

static void adauga_punct_in_coada(const PozitieRobot *pozitie, int unghi_servo_grade, int inclinare_grade, double distanta_cm) {  
    pthread_mutex_lock(&mutex_coada);
    if (coada_numar_elemente == CAPACITATE_COADA_PUNCTE) {
        coada_puncte_pierdute++;
    } else {
        int pozitie_in_coada = (coada_index_inceput + coada_numar_elemente) % CAPACITATE_COADA_PUNCTE;
        coada_puncte_harta[pozitie_in_coada].pozitie_robot = *pozitie;
        coada_puncte_harta[pozitie_in_coada].unghi_servo_grade = unghi_servo_grade;
        coada_puncte_harta[pozitie_in_coada].inclinare_grade = inclinare_grade;
        coada_puncte_harta[pozitie_in_coada].distanta_cm = distanta_cm;
        coada_numar_elemente++;
        pthread_cond_signal(&conditie_coada_nevida);
    }
    pthread_mutex_unlock(&mutex_coada);
}

static void *thread_salvare_harta(void *argument_nefolosit) {   
    (void)argument_nefolosit;
    FILE *fisier_harta = fopen("harta.csv", "w");
    if (!fisier_harta) { perror("harta.csv"); return NULL; }
    fprintf(fisier_harta, "x_cm,y_cm,z_cm,dist_cm,pan_deg,tilt_deg,robot_x,robot_y,robot_th\n");
    long numar_puncte_salvate = 0;
    for (;;) {
        PunctHarta punct;
        pthread_mutex_lock(&mutex_coada);
        while (coada_numar_elemente == 0 && !coada_oprire_ceruta) pthread_cond_wait(&conditie_coada_nevida, &mutex_coada);
        if (coada_numar_elemente == 0 && coada_oprire_ceruta) { pthread_mutex_unlock(&mutex_coada); break; }
        punct = coada_puncte_harta[coada_index_inceput];
        coada_index_inceput = (coada_index_inceput + 1) % CAPACITATE_COADA_PUNCTE;
        coada_numar_elemente--;
        pthread_mutex_unlock(&mutex_coada);

        /* coordonate sferice -> carteziene */
        double unghi_absolut_rad = (punct.pozitie_robot.orientare_grade + unghi_senzor_spre_stanga_grade(punct.unghi_servo_grade)) * GRADE_IN_RADIANI;
        double inclinare_rad = punct.inclinare_grade * GRADE_IN_RADIANI;
        double distanta_orizontala_cm = punct.distanta_cm * cos(inclinare_rad);
        double x_cm = punct.pozitie_robot.x_cm + distanta_orizontala_cm * cos(unghi_absolut_rad);
        double y_cm = punct.pozitie_robot.y_cm + distanta_orizontala_cm * sin(unghi_absolut_rad);
        double z_cm = INALTIME_SENZOR_DE_LA_PODEA_CM + punct.distanta_cm * sin(inclinare_rad);
        fprintf(fisier_harta, "%.1f,%.1f,%.1f,%.1f,%d,%d,%.1f,%.1f,%.1f\n",
                x_cm, y_cm, z_cm, punct.distanta_cm, punct.unghi_servo_grade, punct.inclinare_grade,
                punct.pozitie_robot.x_cm, punct.pozitie_robot.y_cm, punct.pozitie_robot.orientare_grade);
        if (++numar_puncte_salvate % 50 == 0) fflush(fisier_harta);
    }
    fclose(fisier_harta);
    if (coada_puncte_pierdute) fprintf(stderr, "[harta] %ld puncte pierdute (coada plina)\n", coada_puncte_pierdute);
    printf("[harta] %ld puncte salvate in harta.csv\n", numar_puncte_salvate);
    return NULL;
}

/* ===================== 6. THREAD SCANARE ===================== */

static void actualizeaza_perete_cel_mai_apropiat(const PozitieRobot *pozitie, int unghi_servo_grade, double distanta_cm) {   /* era stat_actualizeaza */
    pthread_mutex_lock(&mutex_perete_apropiat);
    if (g_perete_cel_mai_apropiat.activ && distanta_cm < g_perete_cel_mai_apropiat.distanta_cm) {
        g_perete_cel_mai_apropiat.distanta_cm = distanta_cm;
        g_perete_cel_mai_apropiat.directie_grade = pozitie->orientare_grade + unghi_senzor_spre_stanga_grade(unghi_servo_grade);
    }
    pthread_mutex_unlock(&mutex_perete_apropiat);
}

static void *thread_scanare_senzor(void *argument_nefolosit) {  
    (void)argument_nefolosit;
    seteaza_prioritate_timp_real(60);
    const double durata_pas_s = DURATA_TRECERE_COMPLETA_S / NUMAR_PASI_PE_TRECERE;  
    int unghi_servo_grade = UNGHI_SERVO_MINIM_GRADE;    
    int sens_scanare = 1;                               
    int numar_trecere = 0;                             

    pozitioneaza_servo(PIN_SERVO_ORIZONTAL, unghi_servo_grade);
#if HARTA_3D_ACTIVA
    pozitioneaza_servo(PIN_SERVO_VERTICAL, 90.0);
#endif
    dormi_secunde(0.6);

    struct timespec urmatorul_moment;                   
    clock_gettime(CLOCK_MONOTONIC, &urmatorul_moment);

    while (g_programul_ruleaza) {
        if (g_servo_fix_inainte) {
            static int ecouri_invalide_consecutive = 0;                
            static double ultima_distanta_valida_cm = DISTANTA_FARA_OBSTACOL_CM;    
            pozitioneaza_servo(PIN_SERVO_ORIZONTAL, 90.0);
            double distanta_fata_cm = masoara_distanta_ultrasonic_cm();           
            if (distanta_fata_cm < 0.0) {
                ecouri_invalide_consecutive++;
                /* 3 ecouri pierdute dupa ce vedeam ceva aproape = suntem lipiti de perete */
                if (ecouri_invalide_consecutive >= 3 && ultima_distanta_valida_cm < 100.0) distanta_fata_cm = 3.0;
            } else { ecouri_invalide_consecutive = 0; ultima_distanta_valida_cm = distanta_fata_cm; }
            if (distanta_fata_cm > 0.0) {
                int index_unghi_90_grade = (90 - UNGHI_SERVO_MINIM_GRADE) / PAS_UNGHI_SERVO_GRADE;  
                double moment_masuratoare_fata_s = timp_curent_s();                                  
                pthread_mutex_lock(&mutex_scanare);
                g_scanare.distante_cm[index_unghi_90_grade] = distanta_fata_cm;
                g_scanare.momente_masuratorii_s[index_unghi_90_grade] = moment_masuratoare_fata_s;
                g_scanare.moment_ultima_masuratoare_s = moment_masuratoare_fata_s;
                pthread_mutex_unlock(&mutex_scanare);
            }
            dormi_secunde(0.03);
            clock_gettime(CLOCK_MONOTONIC, &urmatorul_moment);
            continue;
        }

        double distanta_cm = masoara_distanta_ultrasonic_cm();      /* servo-ul a avut timp sa ajunga */
        double moment_masuratoare_s = timp_curent_s();
        PozitieRobot pozitie = citeste_pozitie_robot();
        int index_unghi = (unghi_servo_grade - UNGHI_SERVO_MINIM_GRADE) / PAS_UNGHI_SERVO_GRADE;

        if (g_inclinare_servo_grade == 0) {                         /* doar planul orizontal serveste navigarii */
            pthread_mutex_lock(&mutex_scanare);
            g_scanare.distante_cm[index_unghi] = (distanta_cm > 0.0) ? distanta_cm : DISTANTA_FARA_OBSTACOL_CM;
            g_scanare.momente_masuratorii_s[index_unghi] = moment_masuratoare_s;
            g_scanare.moment_ultima_masuratoare_s = moment_masuratoare_s;
            pthread_mutex_unlock(&mutex_scanare);
            if (distanta_cm > 0.0) actualizeaza_perete_cel_mai_apropiat(&pozitie, unghi_servo_grade, distanta_cm);
        }
        if (distanta_cm > 0.0) adauga_punct_in_coada(&pozitie, unghi_servo_grade, g_inclinare_servo_grade, distanta_cm);

        unghi_servo_grade += sens_scanare * PAS_UNGHI_SERVO_GRADE;
        if (unghi_servo_grade >= UNGHI_SERVO_MAXIM_GRADE || unghi_servo_grade <= UNGHI_SERVO_MINIM_GRADE) {
            unghi_servo_grade = (unghi_servo_grade >= UNGHI_SERVO_MAXIM_GRADE) ? UNGHI_SERVO_MAXIM_GRADE : UNGHI_SERVO_MINIM_GRADE;
            sens_scanare = -sens_scanare;
            numar_trecere++;
            pthread_mutex_lock(&mutex_scanare);
            g_scanare.numar_treceri++;
            pthread_mutex_unlock(&mutex_scanare);
#if HARTA_3D_ACTIVA
            {   /* o trecere orizontala, apoi una inclinata, alternativ */
                static const int NIVELE_INCLINARE_GRADE[] = { -25, 25 };   
                g_inclinare_servo_grade = (numar_trecere % 2 == 0) ? 0 : NIVELE_INCLINARE_GRADE[(numar_trecere / 2) % 2];
                pozitioneaza_servo(PIN_SERVO_VERTICAL, 90.0 + g_inclinare_servo_grade);
            }
#endif
        }
        pozitioneaza_servo(PIN_SERVO_ORIZONTAL, unghi_servo_grade);

        long nanosecunde_pas = (long)(durata_pas_s * 1e9);     
        urmatorul_moment.tv_nsec += nanosecunde_pas;
        while (urmatorul_moment.tv_nsec >= 1000000000L) { urmatorul_moment.tv_nsec -= 1000000000L; urmatorul_moment.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &urmatorul_moment, NULL);
    }
    return NULL;
}

/* ===================== 7. THREAD MOTOARE ===================== */

static void *thread_control_motoare(void *argument_nefolosit) {     
    (void)argument_nefolosit;
    seteaza_prioritate_timp_real(50);

    double comanda_curenta[2] = { 0.0, 0.0 };               
    double impuls_pornire_pana_la_s[2] = { 0.0, 0.0 };     
    int motor_activ_anterior[2] = { 0, 0 };                
    int directie_anterioara[2] = { 99, 99 };                
    double factor_umplere_anterior[2] = { -1.0, -1.0 };     
    double franare_pana_la_s = 0.0;                        
    int robotul_merge = 0;                                
    double moment_anterior_s = timp_curent_s();             

    struct timespec urmatorul_moment;                       
    clock_gettime(CLOCK_MONOTONIC, &urmatorul_moment);

    while (g_programul_ruleaza) {
        double moment_curent_s = timp_curent_s();          
        double interval_s = moment_curent_s - moment_anterior_s;   
        moment_anterior_s = moment_curent_s;

        pthread_mutex_lock(&mutex_comanda);
        ComandaMotoare comanda_primita = g_comanda_motoare; 
        pthread_mutex_unlock(&mutex_comanda);
        if (moment_curent_s - comanda_primita.moment_emiterii_s > TIMEOUT_COMANDA_WATCHDOG_S) {     /* watchdog */
            comanda_primita.viteza_stanga = 0.0;
            comanda_primita.viteza_dreapta = 0.0;
            comanda_primita.franare_activa = 1;
        }

        double comanda_tinta[2] = { limiteaza_valoare(comanda_primita.viteza_stanga, -1.0, 1.0),
                                    limiteaza_valoare(comanda_primita.viteza_dreapta, -1.0, 1.0) };    
        double crestere_maxima_pe_pas = CRESTERE_MAXIMA_COMANDA_PE_SECUNDA * interval_s;                
        for (int i = 0; i < 2; i++) {
            if (fabs(comanda_tinta[i]) < 0.03) comanda_curenta[i] = 0.0;
            else comanda_curenta[i] += limiteaza_valoare(comanda_tinta[i] - comanda_curenta[i], -crestere_maxima_pe_pas, crestere_maxima_pe_pas);
        }

        int directie[2] = { 0, 0 };                         
        double factor_umplere[2] = { 0.0, 0.0 };           
        int robot_oprit = (fabs(comanda_curenta[0]) < 0.03 && fabs(comanda_curenta[1]) < 0.03);     

        if (robot_oprit) {
            if (robotul_merge && comanda_primita.franare_activa) franare_pana_la_s = moment_curent_s + DURATA_FRANARE_ACTIVA_S;
            robotul_merge = 0;
            directie[0] = directie[1] = (moment_curent_s < franare_pana_la_s) ? 2 : 0;
            motor_activ_anterior[0] = motor_activ_anterior[1] = 0;
        } else {
            robotul_merge = 1;
            for (int i = 0; i < 2; i++) {
                if (fabs(comanda_curenta[i]) < 0.03) { directie[i] = 0; motor_activ_anterior[i] = 0; continue; }
                directie[i] = (comanda_curenta[i] > 0.0) ? 1 : -1;
                factor_umplere[i] = PWM_MINIM_PORNIRE_MOTOR + (PWM_MAXIM_MOTOR - PWM_MINIM_PORNIRE_MOTOR) * fmin(fabs(comanda_curenta[i]), 1.0);
                if (!motor_activ_anterior[i]) impuls_pornire_pana_la_s[i] = moment_curent_s + DURATA_IMPULS_PORNIRE_S;   /* pornire din repaus */
                if (moment_curent_s < impuls_pornire_pana_la_s[i]) factor_umplere[i] = PWM_IMPULS_PORNIRE;
                motor_activ_anterior[i] = 1;
            }
        }
        for (int i = 0; i < 2; i++) {
            if (directie[i] != directie_anterioara[i] || fabs(factor_umplere[i] - factor_umplere_anterior[i]) > 0.01) {
                aplica_comanda_pe_motor(i, directie[i], factor_umplere[i]);
                directie_anterioara[i] = directie[i];
                factor_umplere_anterior[i] = factor_umplere[i];
            }
        }

        /* odometrie: integreaza comenzile (se poate inlocui cu encodere/giroscop) */
        double viteza_liniara_cm_s = VITEZA_LINIARA_CM_S_LA_COMANDA_MAXIMA * (comanda_curenta[0] + comanda_curenta[1]) / 2.0;           
        double viteza_unghiulara_grade_s = VITEZA_UNGHIULARA_GRADE_S_LA_COMANDA_MAXIMA * (comanda_curenta[1] - comanda_curenta[0]) / 2.0; 
        pthread_mutex_lock(&mutex_pozitie);
        g_pozitie_robot.orientare_grade += viteza_unghiulara_grade_s * interval_s;
        g_pozitie_robot.rotatie_cumulata_grade += viteza_unghiulara_grade_s * interval_s;
        g_pozitie_robot.x_cm += viteza_liniara_cm_s * cos(g_pozitie_robot.orientare_grade * GRADE_IN_RADIANI) * interval_s;
        g_pozitie_robot.y_cm += viteza_liniara_cm_s * sin(g_pozitie_robot.orientare_grade * GRADE_IN_RADIANI) * interval_s;
        g_pozitie_robot.drum_parcurs_cm += fabs(viteza_liniara_cm_s) * interval_s;
        g_pozitie_robot.viteza_cm_s = viteza_liniara_cm_s;
        pthread_mutex_unlock(&mutex_pozitie);

        urmatorul_moment.tv_nsec += 20000000L;                              /* 50 Hz */
        while (urmatorul_moment.tv_nsec >= 1000000000L) { urmatorul_moment.tv_nsec -= 1000000000L; urmatorul_moment.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &urmatorul_moment, NULL);
    }
    return NULL;
}

/* ===================== 8. PERCEPTIE (din scanare) ===================== */

/* distanta libera in fata robotului, prin "coridorul" cat latimea lui, in cm
 * (masuratorile sunt corectate cu cat a inaintat robotul de la masuratoare) */
static double distanta_libera_in_fata_cm(const DateScanare *scanare, double moment_curent_s, double viteza_inainte_cm_s) {   /* era clearance_fata */
    double cea_mai_mica_distanta_cm = DISTANTA_FARA_OBSTACOL_CM;    /* era best */
    for (int i = 0; i < NUMAR_UNGHIURI_MASURATE; i++) {
        double varsta_s = moment_curent_s - scanare->momente_masuratorii_s[i];     /* era age */
        if (varsta_s > VARSTA_MAXIMA_MASURATOARE_S) continue;
        double unghi_rad = unghi_senzor_spre_stanga_grade(UNGHI_SERVO_MINIM_GRADE + i * PAS_UNGHI_SERVO_GRADE) * GRADE_IN_RADIANI;   /* era a */
        double distanta_inainte_cm = scanare->distante_cm[i] * cos(unghi_rad);      /* era fwd */
        double distanta_laterala_cm = scanare->distante_cm[i] * sin(unghi_rad);     /* era lat */
        if (distanta_inainte_cm <= 0.0 || fabs(distanta_laterala_cm) >= SEMILATIME_ROBOT_CU_MARJA_CM) continue;
        double distanta_prezisa_cm = distanta_inainte_cm - fmax(viteza_inainte_cm_s, 0.0) * varsta_s;   /* era pred */
        if (distanta_prezisa_cm < cea_mai_mica_distanta_cm) cea_mai_mica_distanta_cm = distanta_prezisa_cm;
    }
    return cea_mai_mica_distanta_cm + DEPASIRE_SENZOR_FATA_DE_SASIU_CM;
}

/* distanta perpendiculara fata de peretele din stanga (medie pe unghiurile 50..80) */
static double distanta_la_peretele_stang_cm(const DateScanare *scanare, double moment_curent_s) {   
    double suma_distante_cm = 0.0;                      
    int numar_masuratori = 0;                         
    for (int i = 0; i < NUMAR_UNGHIURI_MASURATE; i++) {
        double unghi_stanga_grade = unghi_senzor_spre_stanga_grade(UNGHI_SERVO_MINIM_GRADE + i * PAS_UNGHI_SERVO_GRADE);  
        if (unghi_stanga_grade < 50.0 || unghi_stanga_grade > 80.0) continue;
        if (moment_curent_s - scanare->momente_masuratorii_s[i] > VARSTA_MAXIMA_MASURATOARE_S || scanare->distante_cm[i] > 200.0) continue;
        suma_distante_cm += scanare->distante_cm[i] * sin(unghi_stanga_grade * GRADE_IN_RADIANI);
        numar_masuratori++;
    }
    return numar_masuratori ? suma_distante_cm / numar_masuratori : DISTANTA_FARA_OBSTACOL_CM;
}

static double eroare_rms_profil_scanare(const DateScanare *scanare, const double *profil_referinta_cm, double moment_curent_s) {  
    double suma_patratelor_diferentelor = 0.0;         
    int numar_masuratori = 0;                          
    for (int i = 0; i < NUMAR_UNGHIURI_MASURATE; i++) {
        if (moment_curent_s - scanare->momente_masuratorii_s[i] > VARSTA_MAXIMA_MASURATOARE_S) continue;
        double distanta_curenta_cm = fmin(scanare->distante_cm[i], 200.0);          
        double distanta_referinta_cm = fmin(profil_referinta_cm[i], 200.0);         
        suma_patratelor_diferentelor += (distanta_curenta_cm - distanta_referinta_cm) * (distanta_curenta_cm - distanta_referinta_cm);
        numar_masuratori++;
    }
    return numar_masuratori ? sqrt(suma_patratelor_diferentelor / numar_masuratori) : 1e9;
}

/* ===================== 9. THREAD NAVIGARE ===================== */

static void *thread_navigare(void *argument_nefolosit) {
    (void)argument_nefolosit;
    PozitieRobot pozitie_referinta_faza = citeste_pozitie_robot();      
    PozitieRobot pozitie_start_urmarire = pozitie_referinta_faza;       
    double profil_scanare_la_start[NUMAR_UNGHIURI_MASURATE];           
    double moment_start_faza_s = timp_curent_s();                       
    double moment_start_urmarire_s = moment_start_faza_s;               
    double directie_cel_mai_apropiat_perete_grade = 0.0;                
    int robot_blocat_in_fata = 0;                                       
    int stare_deblocare = 0;                                            
    double asteapta_scanare_pana_la_s = 0.0;                            
    double rotatie_cumulata_la_inceput_deblocare_grade = 0.0;           
    int faza_anterioara = -1;                                          

    for (int i = 0; i < NUMAR_UNGHIURI_MASURATE; i++) profil_scanare_la_start[i] = DISTANTA_FARA_OBSTACOL_CM;

    while (g_programul_ruleaza) {
        double moment_curent_s = timp_curent_s();                      
        PozitieRobot pozitie = citeste_pozitie_robot();                 
        DateScanare scanare;                                            
        copiaza_date_scanare(&scanare);
        double distanta_libera_fata_cm = distanta_libera_in_fata_cm(&scanare, moment_curent_s, pozitie.viteza_cm_s);   

        if (g_faza_curenta != faza_anterioara) {                        /* intrare intr-o faza noua */
            printf("[nav] faza: %s\n", NUME_FAZE[g_faza_curenta]);
            faza_anterioara = g_faza_curenta;
            moment_start_faza_s = moment_curent_s;
            pozitie_referinta_faza = pozitie;
        }
        /* siguranta: daca scanarea s-a oprit, ne oprim si noi */
        if (g_faza_curenta >= FAZA_APROPIE && g_faza_curenta != FAZA_GATA && moment_curent_s - scanare.moment_ultima_masuratoare_s > 1.0) {
            comanda_franare();
            dormi_secunde(0.05);
            continue;
        }

        switch (g_faza_curenta) {

        case FAZA_ASTEAPTA:                     /* asteapta prima scanare completa */
            seteaza_comanda_motoare(0, 0, 1);
            if (scanare.numar_treceri >= 1) {
                pthread_mutex_lock(&mutex_perete_apropiat);
                g_perete_cel_mai_apropiat.activ = 1;
                g_perete_cel_mai_apropiat.distanta_cm = 1e9;
                g_perete_cel_mai_apropiat.directie_grade = 0.0;
                pthread_mutex_unlock(&mutex_perete_apropiat);
                g_faza_curenta = FAZA_SCAN_360;
            }
            break;

        case FAZA_SCAN_360:                     /* rotire completa pe loc, din centrul camerei */
            seteaza_comanda_motoare(-VITEZA_ROTIRE_PE_LOC, VITEZA_ROTIRE_PE_LOC, 0);
            if (pozitie.rotatie_cumulata_grade - pozitie_referinta_faza.rotatie_cumulata_grade >= 360.0 || moment_curent_s - moment_start_faza_s > 40.0) {
                comanda_franare();
                pthread_mutex_lock(&mutex_perete_apropiat);
                g_perete_cel_mai_apropiat.activ = 0;
                directie_cel_mai_apropiat_perete_grade = g_perete_cel_mai_apropiat.directie_grade;
                pthread_mutex_unlock(&mutex_perete_apropiat);
                dormi_secunde(0.5);
                g_faza_curenta = FAZA_CAUTA_PERETE;
            }
            break;

        case FAZA_CAUTA_PERETE: {               /* se orienteaza spre cel mai apropiat perete */
            double diferenta_unghi_grade = normalizeaza_unghi_180(directie_cel_mai_apropiat_perete_grade - pozitie.orientare_grade);   /* era delta */
            if (fabs(diferenta_unghi_grade) < 8.0 || moment_curent_s - moment_start_faza_s > 40.0) {
                comanda_franare();
                dormi_secunde(1.7);             /* scanare proaspata inainte de apropiere */
                g_servo_fix_inainte = 1;
                g_faza_curenta = FAZA_APROPIE;
            } else if (diferenta_unghi_grade > 0.0) seteaza_comanda_motoare(-VITEZA_ROTIRE_PE_LOC, VITEZA_ROTIRE_PE_LOC, 0);
            else                                    seteaza_comanda_motoare(VITEZA_ROTIRE_PE_LOC, -VITEZA_ROTIRE_PE_LOC, 0);
            break;
        }

        case FAZA_APROPIE:                      /* mers lent, oprire la DISTANTA_OPRIRE_FATA_DE_OBSTACOL_CM de perete */
            if (distanta_libera_fata_cm <= DISTANTA_OPRIRE_FATA_DE_OBSTACOL_CM + DISTANTA_PARCURSA_IN_TIMPUL_FRANARII_CM) {
                comanda_franare();
                dormi_secunde(0.6);
                g_servo_fix_inainte = 0;
                g_faza_curenta = FAZA_RETRAGE;
            } else if (pozitie.drum_parcurs_cm - pozitie_referinta_faza.drum_parcurs_cm > 500.0) {
                printf("[nav] nu am gasit niciun perete in 5 m, ma opresc\n");
                g_servo_fix_inainte = 0;
                g_faza_curenta = FAZA_GATA;
            } else {
                double factor_apropiere = limiteaza_valoare((distanta_libera_fata_cm - DISTANTA_OPRIRE_FATA_DE_OBSTACOL_CM) /
                                                            (DISTANTA_INCEPUT_INCETINIRE_CM - DISTANTA_OPRIRE_FATA_DE_OBSTACOL_CM), 0.0, 1.0);   
                double viteza_inainte = VITEZA_MINIMA_APROPIERE + (VITEZA_CROAZIERA - VITEZA_MINIMA_APROPIERE) * factor_apropiere;               
                seteaza_comanda_motoare(viteza_inainte, viteza_inainte, 0);
            }
            break;

        case FAZA_RETRAGE:                      /* un pic inapoi, ca sa aiba loc la rotire */
            if (pozitie.drum_parcurs_cm - pozitie_referinta_faza.drum_parcurs_cm >= DISTANTA_RETRAGERE_CM || moment_curent_s - moment_start_faza_s > 10.0) {
                comanda_franare();
                dormi_secunde(0.5);
                g_faza_curenta = FAZA_ALINIEAZA;
            } else seteaza_comanda_motoare(-VITEZA_MINIMA_APROPIERE, -VITEZA_MINIMA_APROPIERE, 0);
            break;

        case FAZA_ALINIEAZA:                    /* peretele ajunge in stanga robotului */
            seteaza_comanda_motoare(VITEZA_ROTIRE_PE_LOC, -VITEZA_ROTIRE_PE_LOC, 0);
            if (pozitie.rotatie_cumulata_grade - pozitie_referinta_faza.rotatie_cumulata_grade <= -UNGHI_ALINIERE_LA_PERETE_GRADE || moment_curent_s - moment_start_faza_s > 20.0) {
                comanda_franare();
                dormi_secunde(1.7);             /* asteapta o scanare noua */
                DateScanare scanare_noua;     
                copiaza_date_scanare(&scanare_noua);
                memcpy(profil_scanare_la_start, scanare_noua.distante_cm, sizeof profil_scanare_la_start);
                pozitie_start_urmarire = citeste_pozitie_robot();
                moment_start_urmarire_s = timp_curent_s();
                robot_blocat_in_fata = 0;
                g_faza_curenta = FAZA_URMARESTE_PERETE;
            }
            break;

        case FAZA_URMARESTE_PERETE: {
            double distanta_perete_stang_cm = distanta_la_peretele_stang_cm(&scanare, moment_curent_s);    

            /* --- a terminat tura? (inchiderea buclei) --- */
            double rotatie_in_tura_grade = pozitie.rotatie_cumulata_grade - pozitie_start_urmarire.rotatie_cumulata_grade;    
            double drum_in_tura_cm = pozitie.drum_parcurs_cm - pozitie_start_urmarire.drum_parcurs_cm;                         
            double distanta_fata_de_start_cm = hypot(pozitie.x_cm - pozitie_start_urmarire.x_cm,
                                                     pozitie.y_cm - pozitie_start_urmarire.y_cm);                             
            double eroare_profil_cm = eroare_rms_profil_scanare(&scanare, profil_scanare_la_start, moment_curent_s);          
            int tura_inchisa = (rotatie_in_tura_grade >= ROTATIE_MINIMA_TURA_COMPLETA_GRADE && drum_in_tura_cm >= DRUM_MINIM_TURA_CM &&
                                (distanta_fata_de_start_cm < RAZA_INCHIDERE_BUCLA_CM || eroare_profil_cm < TOLERANTA_PROFIL_SCANARE_CM));  
            if (tura_inchisa || moment_curent_s - moment_start_urmarire_s > TIMP_MAXIM_TURA_S) {
                printf("[nav] tura completa: unghi %.0f grd, drum %.0f cm, distanta de start %.0f cm, profil %.0f cm\n",
                       rotatie_in_tura_grade, drum_in_tura_cm, distanta_fata_de_start_cm, eroare_profil_cm);
                comanda_franare();
                dormi_secunde(0.5);
                g_faza_curenta = FAZA_GATA;
                break;
            }

            /* --- blocat in fata: rotire in pasi mici, cu scanare intre pasi --- */
            if (robot_blocat_in_fata) {
                if (stare_deblocare == 0) {
                    seteaza_comanda_motoare(VITEZA_ROTIRE_PE_LOC, -VITEZA_ROTIRE_PE_LOC, 0);        /* spre dreapta */
                    if (pozitie.rotatie_cumulata_grade - rotatie_cumulata_la_inceput_deblocare_grade <= -PAS_ROTIRE_CAND_BLOCAT_GRADE) {
                        comanda_franare();
                        asteapta_scanare_pana_la_s = moment_curent_s + DURATA_TRECERE_COMPLETA_S * 1.1;
                        stare_deblocare = 1;
                    }
                } else {
                    comanda_franare();
                    if (moment_curent_s >= asteapta_scanare_pana_la_s) {
                        if (distanta_libera_fata_cm > DISTANTA_INCEPUT_INCETINIRE_CM * 0.8) robot_blocat_in_fata = 0;
                        else { stare_deblocare = 0; rotatie_cumulata_la_inceput_deblocare_grade = pozitie.rotatie_cumulata_grade; }
                    }
                }
                break;
            }
            if (distanta_libera_fata_cm <= DISTANTA_OPRIRE_FATA_DE_OBSTACOL_CM + DISTANTA_PARCURSA_IN_TIMPUL_FRANARII_CM) {
                robot_blocat_in_fata = 1; stare_deblocare = 0; rotatie_cumulata_la_inceput_deblocare_grade = pozitie.rotatie_cumulata_grade;
                comanda_franare();
                break;
            }

            /* --- mers normal pe langa perete, cu curbe line --- */
            double factor_apropiere = limiteaza_valoare((distanta_libera_fata_cm - DISTANTA_OPRIRE_FATA_DE_OBSTACOL_CM) /
                                                        (DISTANTA_INCEPUT_INCETINIRE_CM - DISTANTA_OPRIRE_FATA_DE_OBSTACOL_CM), 0.0, 1.0);       
            double viteza_inainte = VITEZA_MINIMA_APROPIERE + (VITEZA_CROAZIERA - VITEZA_MINIMA_APROPIERE) * factor_apropiere;                   
            double directie_virare;                                                                    
            if (distanta_perete_stang_cm > DISTANTA_PERETE_STANG_PIERDUT_CM) { directie_virare = VIRAJ_CAUTARE_PERETE_PIERDUT; viteza_inainte *= 0.8; }
            else directie_virare = CASTIG_PROPORTIONAL_PERETE * limiteaza_valoare(
                     (distanta_perete_stang_cm - DISTANTA_DORITA_FATA_DE_PERETE_STANG_CM) / DISTANTA_DORITA_FATA_DE_PERETE_STANG_CM, -1.0, 1.0);
            directie_virare -= CASTIG_EVITARE_OBSTACOL_FATA * (1.0 - factor_apropiere);                 /* ceva in fata => spre dreapta */
            directie_virare = limiteaza_valoare(directie_virare, -1.0, 1.0);
            seteaza_comanda_motoare(limiteaza_valoare(viteza_inainte * (1.0 - CASTIG_CURBARE_ARC * directie_virare), 0.0, 1.0),
                                    limiteaza_valoare(viteza_inainte * (1.0 + CASTIG_CURBARE_ARC * directie_virare), 0.0, 1.0), 0);
            break;
        }

        case FAZA_GATA:
        default:
            comanda_franare();
            dormi_secunde(0.5);
            g_programul_ruleaza = 0;
            break;
        }
        dormi_secunde(0.05);                                    /* 20 Hz */
    }
    return NULL;
}

/* ===================== 10. MAIN ===================== */

static void afiseaza_stare(void) {                              
    PozitieRobot pozitie = citeste_pozitie_robot();
    DateScanare scanare;
    copiaza_date_scanare(&scanare);
    double moment_curent_s = timp_curent_s();
    printf("[%-16s] x=%6.1f y=%6.1f th=%6.1f | fata=%5.1f perete_st=%5.1f | treceri=%d\n",
           NUME_FAZE[g_faza_curenta], pozitie.x_cm, pozitie.y_cm, normalizeaza_unghi_180(pozitie.orientare_grade),
           distanta_libera_in_fata_cm(&scanare, moment_curent_s, pozitie.viteza_cm_s),
           distanta_la_peretele_stang_cm(&scanare, moment_curent_s), scanare.numar_treceri);
}

static void initializeaza_stari(void) {                        
    for (int i = 0; i < NUMAR_UNGHIURI_MASURATE; i++) {
        g_scanare.distante_cm[i] = DISTANTA_FARA_OBSTACOL_CM;
        g_scanare.momente_masuratorii_s[i] = -1000.0;
    }
    g_scanare.moment_ultima_masuratoare_s = timp_curent_s();
    g_comanda_motoare.moment_emiterii_s = timp_curent_s();
}

int main(int numar_argumente, char **argumente) {              
    const char *mod_functionare = (numar_argumente > 1) ? argumente[1] : "run";    
    int scanare_activa  = !strcmp(mod_functionare, "senzor") || !strcmp(mod_functionare, "run");    
    int motoare_active  = !strcmp(mod_functionare, "calib-mers") || !strcmp(mod_functionare, "calib-viraj") || !strcmp(mod_functionare, "run");   /* era motor_on */
    int navigare_activa = !strcmp(mod_functionare, "run");                                           
    if (!scanare_activa && !motoare_active) {
        fprintf(stderr, "Folosire: %s senzor | calib-mers | calib-viraj | run\n", argumente[0]);
        return 1;
    }

    setvbuf(stdout, NULL, _IOLBF, 0);
    struct sigaction actiune_semnal;                           
    memset(&actiune_semnal, 0, sizeof actiune_semnal);
    actiune_semnal.sa_handler = la_primire_semnal;
    sigaction(SIGINT, &actiune_semnal, NULL);
    sigaction(SIGTERM, &actiune_semnal, NULL);
    sigaction(SIGHUP, &actiune_semnal, NULL);

    if (!initializeaza_hardware()) return 1;
    initializeaza_stari();

    pthread_t id_thread_scanare, id_thread_motoare, id_thread_navigare, id_thread_harta;  
    if (scanare_activa)  { pthread_create(&id_thread_harta, NULL, thread_salvare_harta, NULL);
                           pthread_create(&id_thread_scanare, NULL, thread_scanare_senzor, NULL); }
    if (motoare_active)  pthread_create(&id_thread_motoare, NULL, thread_control_motoare, NULL);
    if (navigare_activa) pthread_create(&id_thread_navigare, NULL, thread_navigare, NULL);

    printf("Mod: %s. Oprire cu Ctrl+C.\n", mod_functionare);

    if (!strcmp(mod_functionare, "calib-mers")) {
        printf("Pune robotul pe podea, cu 1 m liber in fata. Pornesc in 3 s.\n");
        dormi_secunde(3.0);
        double moment_start_s = timp_curent_s();                
        while (g_programul_ruleaza && timp_curent_s() - moment_start_s < 3.0) { seteaza_comanda_motoare(VITEZA_CROAZIERA, VITEZA_CROAZIERA, 0); dormi_secunde(0.05); }
        comanda_franare();
        dormi_secunde(1.0);
        printf("Masoara distanta D parcursa (cm). Apoi: K_VEL_CM_S = D / (3.0 * %.2f) = D / %.2f\n",
               VITEZA_CROAZIERA, 3.0 * VITEZA_CROAZIERA);
        g_programul_ruleaza = 0;
    } else if (!strcmp(mod_functionare, "calib-viraj")) {
        printf("Marcheaza directia robotului pe podea. Pornesc in 3 s.\n");
        dormi_secunde(3.0);
        double moment_start_s = timp_curent_s();              
        while (g_programul_ruleaza && timp_curent_s() - moment_start_s < 4.0) { seteaza_comanda_motoare(-VITEZA_ROTIRE_PE_LOC, VITEZA_ROTIRE_PE_LOC, 0); dormi_secunde(0.05); }
        comanda_franare();
        dormi_secunde(1.0);
        printf("Masoara unghiul A rotit (grade). Apoi: K_YAW_DEG_S = A / (4.0 * %.2f) = A / %.2f\n",
               VITEZA_ROTIRE_PE_LOC, 4.0 * VITEZA_ROTIRE_PE_LOC);
        g_programul_ruleaza = 0;
    } else {
        while (g_programul_ruleaza) {
            dormi_secunde(0.5);
            if (scanare_activa) afiseaza_stare();
        }
    }

    /* ---- oprire ordonata ---- */
    g_programul_ruleaza = 0;
    if (navigare_activa) pthread_join(id_thread_navigare, NULL);
    if (motoare_active)  pthread_join(id_thread_motoare, NULL);
    if (scanare_activa)  pthread_join(id_thread_scanare, NULL);
    if (motoare_active) {                                       /* franare finala, apoi liber */
        aplica_comanda_pe_motor(0, 2, 1.0); aplica_comanda_pe_motor(1, 2, 1.0);
        dormi_secunde(0.2);
        aplica_comanda_pe_motor(0, 0, 0.0); aplica_comanda_pe_motor(1, 0, 0.0);
    }
    if (scanare_activa) {
        pthread_mutex_lock(&mutex_coada);
        coada_oprire_ceruta = 1;
        pthread_cond_broadcast(&conditie_coada_nevida);
        pthread_mutex_unlock(&mutex_coada);
        pthread_join(id_thread_harta, NULL);
        pozitioneaza_servo(PIN_SERVO_ORIZONTAL, 90.0);
        dormi_secunde(0.4);
        opreste_servo(PIN_SERVO_ORIZONTAL);
#if HARTA_3D_ACTIVA
        opreste_servo(PIN_SERVO_VERTICAL);
#endif
    }
    lgGpiochipClose(g_handle_gpio);
    printf("Oprit curat.\n");
    return 0;
}