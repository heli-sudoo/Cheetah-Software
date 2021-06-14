#include "MHPCCost.h"

template <typename T>
WBCost<T>::WBCost(T dt) : Cost<T, xsize_WB, usize_WB, ysize_WB>(dt, 4)
{
    this->_Q = new MatMN<T, xsize_WB, xsize_WB>[4];
    this->_R = new MatMN<T, usize_WB, usize_WB>[4];
    this->_S = new MatMN<T, ysize_WB, ysize_WB>[4];
    this->_Qf = new MatMN<T, xsize_WB, xsize_WB>[4];
    set_weighting_matrices();
}

template <typename T>
FBCost<T>::FBCost(T dt) : Cost<T, xsize_FB, usize_FB, ysize_FB>(dt, 4)
{
    this->_Q = new MatMN<T, xsize_FB, xsize_FB>[4];
    this->_R = new MatMN<T, usize_FB, usize_FB>[4];
    this->_S = new MatMN<T, ysize_FB, ysize_FB>[4];
    this->_Qf = new MatMN<T, xsize_FB, xsize_FB>[4];
    set_weighting_matrices();
}

template <typename T>
void WBCost<T>::set_weighting_matrices()
{
    // Set weighting matrices for WB running and terminal cost
    VecM<T, xsize_WB> q, qf[4];
    VecM<T, usize_WB> r[4];
    VecM<T, ysize_WB> s[4];
    q << 10, 10, 5, 4, 4, 4, 4, 2, 1, .01, 6, 6, 6, 6;

    qf[0] << 15, 20, 8, 3, 3, 3, 3, 3, 2, 0.01, 5, 5, 0.01, 0.01;
    qf[1] << 15, 20, 8, 3, 3, 3, 3, 3, 2, 0.01, 5, 5, 5, 5;
    qf[2] << 15, 20, 8, 3, 3, 3, 3, 3, 2, 0.01, 0.01, 0.01, 5, 5;
    qf[3] << 15, 20, 8, 3, 3, 3, 3, 3, 2, 0.01, 5, 5, 5, 5;

    r[0] << 5, 5, 1, 1;
    r[1].setOnes();
    r[2] << 1, 1, 5, 5;
    r[3].setOnes();

    std::fill(&s[0], &s[3], VecM<T, ysize_WB>(0, 0, 0, 0));
    s[0].tail(2) << 0.3, 0.3;
    s[2].head(2) << 0.15, 0.15;

    for (size_t i = 0; i < this->_n_modes; i++)
    {
        this->_Q[i] = 0.01 * q.asDiagonal();
        this->_R[i] = 0.5 * r[i].asDiagonal();
        this->_S[i] = s[i].asDiagonal();
        this->_Qf[i] = 100 * qf[i].asDiagonal();
    }
}

template <typename T>
void FBCost<T>::set_weighting_matrices()
{
    VecM<T, xsize_FB> q, qf;
    VecM<T, usize_FB> r[4];
    q.setZero();
    qf.setZero();
    std::memset(&r[0], 0, 4 * sizeof(VecM<T, usize_FB>));
    q << 5, 10, 5, 2, 1, 0.01;
    qf << 15, 20, 8, 3, 1, 0.01;
    r[0].tail(2) << 0.01, 0.01;
    r[2].head(2) << 0.01, 0.01;

    for (size_t i = 0; i < 4; i++)
    {
        this->_Q[i] = 0.01 * q.asDiagonal();
        this->_R[i] = r[i].asDiagonal();
        this->_Qf[i] = 100 * qf.asDiagonal();
        this->_S[i].setZero();
    }
}

template class WBCost<casadi_real>;
template class FBCost<casadi_real>;
