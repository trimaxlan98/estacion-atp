/* Arnes de pruebas minimo (sin dependencias): un ejecutable = una suite de
 * ctest. Unity (que trae ESP-IDF) haria lo mismo; esto evita bajar nada para
 * correr en el host. CHECK imprime la linea que falla y sigue; el codigo de
 * salida es el numero de fallos (0 = todo bien). */
#ifndef MINITEST_H
#define MINITEST_H

#include <math.h>
#include <stdio.h>

static int mt_fallos = 0;
static int mt_checks = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        mt_checks++;                                                             \
        if (!(cond)) {                                                           \
            mt_fallos++;                                                         \
            printf("  FALLA %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
        }                                                                        \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                    \
    do {                                                                         \
        double mt_a = (double)(a), mt_b = (double)(b), mt_t = (double)(tol);     \
        mt_checks++;                                                             \
        if (!(fabs(mt_a - mt_b) <= mt_t)) {                                      \
            mt_fallos++;                                                         \
            printf("  FALLA %s:%d: %s = %.9g, esperado %.9g +- %.3g\n",          \
                   __FILE__, __LINE__, #a, mt_a, mt_b, mt_t);                    \
        }                                                                        \
    } while (0)

#define RUN(fn)                                                                  \
    do {                                                                         \
        int mt_antes = mt_fallos;                                                \
        fn();                                                                    \
        printf("%s %s\n", mt_fallos == mt_antes ? "[ ok ]" : "[FAIL]", #fn);     \
    } while (0)

#define MT_FIN()                                                                 \
    do {                                                                         \
        printf("%d comprobaciones, %d fallos\n", mt_checks, mt_fallos);          \
        return mt_fallos ? 1 : 0;                                                \
    } while (0)

#endif
