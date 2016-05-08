#include "dlnd/localizer.h"

Localizer::Localizer(const int idTurret, const int nbTurret){
    int id = 6*idTurret+3;
    m_v = new ibex::Variable(3+6*nbTurret);
    m_N1 = new ibex::NumConstraint(*m_v,((*m_v)[0]-(*m_v)[id])*(*m_v)[id+4]-((*m_v)[1]-(*m_v)[id+1])*(*m_v)[id+3]=0);
    m_N2 = new ibex::NumConstraint(*m_v,((*m_v)[1]-(*m_v)[id+1])*(*m_v)[id+5]-((*m_v)[2]-(*m_v)[id+2])*(*m_v)[id+4]=0);
    m_N3 = new ibex::NumConstraint(*m_v,((*m_v)[0]-(*m_v)[id])*(*m_v)[id+3]+((*m_v)[1]-(*m_v)[id+1])*(*m_v)[id+4]+((*m_v)[2]-(*m_v)[id+2])*(*m_v)[id+5]>0);
    m_c1 = new ibex::CtcFwdBwd(*m_N1);
    m_c2 = new ibex::CtcFwdBwd(*m_N2);
    m_c3 = new ibex::CtcFwdBwd(*m_N3);
    m_ca = new ibex::CtcCompo(*m_c1,*m_c2);
    m_C = new ibex::CtcCompo(*m_ca,*m_c3);
}

Localizer::~Localizer(){
    delete(m_v);
    m_v = NULL;
    delete(m_N1);
    m_N1 = NULL;
    delete(m_N2);
    m_N2 = NULL;
    delete(m_N3);
    m_N3 = NULL;
    delete(m_c1);
    m_c1 = NULL;
    delete(m_c2);
    m_c2 = NULL;
    delete(m_c3);
    m_c3 = NULL;
    delete(m_ca);
    m_ca = NULL;
    delete(m_C);
    m_C = NULL;
}

ibex::CtcCompo* Localizer::getCtc(){
    return m_C;
}
