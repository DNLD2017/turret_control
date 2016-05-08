#ifndef LOCALIZER_H
#define LOCALIZER_H

#include "ibex.h"

class Localizer{
public:
    Localizer(const int idTurret, const int nbTurret);
    ~Localizer();
    ibex::CtcCompo* getCtc();
private:
    ibex::Variable *m_v;
    ibex::NumConstraint *m_N1, *m_N2, *m_N3;
    ibex::CtcFwdBwd *m_c1, *m_c2, *m_c3;
    ibex::CtcCompo *m_C, *m_ca;
};

#endif // LOCALIZATION_H
