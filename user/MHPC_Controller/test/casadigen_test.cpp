#include "MHPC_CPPTypes.h"
#include "CasadiGen.h"

int main(int argc, char const *argv[])
{
	VecM<casadi_real, 14> x;
	VecM<casadi_real, 4> u;
	MatMN<casadi_real, 2, 7> J;
	MatMN<casadi_real, 2, 7> Jd;
	x.setZero();
	u.setZero();
	J.setZero();
	Jd.setZero();

	vector<casadi_real* > arg = {x.data()};
	vector<casadi_real* > res = {J.data(), Jd.data()};

	casadi_interface(arg, res, x.size(), Jacob_F, Jacob_F_sparsity_out, Jacob_F_work);
	return 0;
}
