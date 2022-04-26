
#include "utility.hpp"
#include "particle_systems.hpp"
#include "integrators.hpp"

void eulerStep(ParticleSystem& ps, float step) {
	// YOUR CODE HERE (R1)
	// Implement an Euler integrator.
	State init = ps.state();
	State now;
	State f = ps.evalF(init);
	// New state value of every praticles
	for (int i = 0; i != init.size(); i++) {
		now.push_back(init[i] + step * f[i]);
	}
	// Set the state
	ps.set_state(now);
};

void trapezoidStep(ParticleSystem& ps, float step) {
	// YOUR CODE HERE (R3)
	// Implement a trapezoid integrator.
	State init = ps.state();
	// f0 at the current state, f0=f(X, t)
	State f0 = ps.evalF(init);
	// First call the euler method
	eulerStep(ps, step);
	// f1 after an Euler step, f1=f(X+hf0, t+h)
	State f1 = ps.evalF(ps.state());
	State now;
	// Then compute in Trapezoidal approach: 
	// X(t+h) = X+h/2*(f0+f1)
	for (int i = 0; i != init.size(); i++) {
		now.push_back(init[i] + (step / 2) * (f0[i] + f1[i]));
	}
	// Set the state
	ps.set_state(now);

}

void midpointStep(ParticleSystem& ps, float step) {
	const auto& x0 = ps.state();
	auto n = x0.size();
	auto f0 = ps.evalF(x0);
	auto xm = State(n), x1 = State(n);
	for (auto i = 0u; i < n; ++i) {
		xm[i] = x0[i] + (0.5f * step) * f0[i];
	}
	auto fm = ps.evalF(xm);
	for (auto i = 0u; i < n; ++i) {
		x1[i] = x0[i] + step * fm[i];
	}
	ps.set_state(x1);
}

void rk4Step(ParticleSystem& ps, float step) {
	// EXTRA: Implement the RK4 Runge-Kutta integrator.
	State init = ps.state();
	State now = ps.state();
	State f = ps.evalF(init);
	State k1, k2, k3, k4;
	k1 = k2 = k3 = k4 = ps.state();
	State temp1, temp2, temp3;
	temp1 = temp2 = temp3 = ps.state();
	for (int i = 0; i != init.size(); i++) {
		k1[i] = step * f[i];
		temp1[i] = init[i] + k1[i] * 0.5;
	}
	State temp1F = ps.evalF(temp1);
	for (int i = 0; i != init.size(); i++) {
		k2[i] = step * temp1F[i];
		temp2[i] = init[i] + k2[i] * 0.5;
	}
	State temp2F = ps.evalF(temp2);
	for (int i = 0; i != init.size(); i++) {
		k3[i] = step * temp2F[i];
		temp3[i] = init[i] + k3[i];
	}
	State temp3F = ps.evalF(temp3);
	for (int i = 0; i != init.size(); i++) {
		k4[i] = step * temp3F[i];
		now[i] = init[i] + (k1[i] + k2[i] * 2 + k3[i] * 2 + k4[i]) / 6;
	}
	ps.set_state(now);

}

#ifdef EIGEN_SPARSECORE_MODULE_H

void implicit_euler_step(ParticleSystem& ps, float step, SparseMatrix& J, SparseLU& solver, bool initial) {
	// EXTRA: Implement the implicit Euler integrator. (Note that the related formula on page 134 on the lecture slides is missing a 'h'; the formula should be (I-h*Jf(Yi))DY=-F(Yi))
}

void implicit_midpoint_step(ParticleSystem& ps, float step, SparseMatrix& J, SparseLU& solver, bool initial) {
	// EXTRA: Implement the implicit midpoint integrator.
}

void crank_nicolson_step(ParticleSystem & ps, float step, SparseMatrix & J, SparseLU & solver, bool initial) {
		// EXTRA: Implement the crank-nicolson integrator.
}
#endif
