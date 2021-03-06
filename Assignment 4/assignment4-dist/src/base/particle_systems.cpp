#include "particle_systems.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <numeric>

using namespace std;
using namespace FW;

namespace {

	inline Vec3f fGravity(float mass) {
		return Vec3f(0, -9.8f * mass, 0);
	}

	// force acting on particle at pos1 due to spring attached to pos2 at the other end
	inline Vec3f fSpring(const Vec3f& pos1, const Vec3f& pos2, float k, float rest_length) {
		// YOUR CODE HERE (R2)
		//return Vec3f(0);
		// Compute distance
		float d = FW::sqrt(FW::pow((pos1.x - pos2.x), 2) + FW::pow((pos1.y - pos2.y), 2) + FW::pow((pos1.z - pos2.z), 2));
		Vec3f forceS = k * (rest_length - d) * ((pos1 - pos2) / d);
		return forceS;
	}

	inline Vec3f fDrag(const Vec3f& v, float k) {
		// YOUR CODE HERE (R2)
		//return Vec3f(0);
		Vec3f forceD = -k * v;
		return forceD;
	}

} // namespace

void SimpleSystem::reset() {
	current_state_ = State(1, Vec3f(0, radius_, 0));
}

State SimpleSystem::evalF(const State& state) const {
	State f(1, Vec3f(-state[0].y, state[0].x, 0));
	return f;
}

#ifdef EIGEN_SPARSECORE_MODULE_H
// using the implicit Euler method, the simple system should converge towards origin -- as opposed to the explicit Euler, which diverges outwards from the origin.
void SimpleSystem::evalJ(const State&, SparseMatrix& result, bool initial) const {
	if (initial) {
		result.coeffRef(1, 0) = 1.0f;
		result.coeffRef(0, 1) = -1.0f;
	}
}
#endif

Points SimpleSystem::getPoints() {
	return Points(1, current_state_[0]);
}

Lines SimpleSystem::getLines() {
	static const auto n_lines = 50u;
	auto l = Lines(n_lines * 2);
	const auto angle_incr = 2 * FW_PI / n_lines;
	for (auto i = 0u; i < n_lines; ++i) {
		l[2 * i] = l[2 * i + 1] =
			Vec3f(radius_ * FW::sin(angle_incr * i), radius_ * FW::cos(angle_incr * i), 0);
	}
	rotate(l.begin(), l.begin() + 1, l.end());
	return l;
}

void SpringSystem::reset() {
	const auto start_pos = Vec3f(0.1f, -0.5f, 0.0f);
	const auto spring_k = 30.0f;
	const auto rest_length = 0.5f;
	current_state_ = State(4);
	// YOUR CODE HERE (R2)
	// Set the initial state for a particle system with one particle fixed
	// at origin and another particle hanging off the first one with a spring.
	// Place the second particle initially at start_pos.
	const auto origin_pos = Vec3f(0.0f, 0.0f, 0.0f);
	spring_ = Spring(0, 1, spring_k, rest_length);
	current_state_[0] = origin_pos;
	current_state_[1] = Vec3f(0.0f, 0.0f, 0.0f);
	current_state_[2] = start_pos;
	current_state_[3] = Vec3f(0.0f, 0.0f, 0.0f);
}

State SpringSystem::evalF(const State& state) const {
	const auto drag_k = 0.5f;
	const auto mass = 1.0f;
	State f(4);
	// YOUR CODE HERE (R2)
	// Return a derivative for the system as if it was in state "state".
	// You can use the fGravity, fDrag and fSpring helper functions for the forces.
	Vec3f force;
	Vec3f forceG, forceS, forceD;
	forceG = fGravity(mass);
	forceS = fSpring(state[2], state[0], spring_.k, spring_.rlen);
	forceD = fDrag(state[3], drag_k);
	force = forceG + forceS + forceD;
	f[0] = state[1];
	f[1] = 0;
	f[2] = state[3];
	f[3] = force;
	return f;
}

#ifdef EIGEN_SPARSECORE_MODULE_H

// This is a very useful read for the Jacobians of the spring forces. It deals with spring damping as well, we don't do that -- our drag is simply a linear damping of velocity (that results in some constants in the Jacobian).
// http://blog.mmacklin.com/2012/05/04/implicitsprings/

void SpringSystem::evalJ(const State& state, SparseMatrix& result, bool initial) const {
	const auto drag_k = 0.5f;
	const auto mass = 1.0f;
	// EXTRA: Evaluate the Jacobian into the 'result' matrix here. Only the free end of the spring should have any nonzero values related to it.
}
#endif

Points SpringSystem::getPoints() {
	auto p = Points(2);
	p[0] = current_state_[0]; p[1] = current_state_[2];
	return p;
}

Lines SpringSystem::getLines() {
	auto l = Lines(2);
	l[0] = current_state_[0]; l[1] = current_state_[2];
	return l;
}

void PendulumSystem::reset() {
	const auto spring_k = 1000.0f;
	const auto start_point = Vec3f(0);
	const auto end_point = Vec3f(0.05, -1.5, 0);
	current_state_ = State(2 * n_);
	// YOUR CODE HERE (R4)
	// Set the initial state for a pendulum system with n_ particles
	// connected with springs into a chain from start_point to end_point with uniform intervals.
	// The rest length of each spring is its length in this initial configuration.
	float rest_length = FW::sqrt(FW::pow((end_point.x - start_point.x), 2) + FW::pow((end_point.y - start_point.y), 2) + FW::pow((end_point.z - start_point.z), 2));
	// Scale 
	float scale_x = (end_point.x - start_point.x) / (n_ - 1);
	float scale_y = (end_point.y - start_point.y) / (n_ - 1);
	int i;
	for (i = 0; i != n_ - 1; i++) {
		current_state_[2 * i + 1] = Vec3f(0.0f, 0.0f, 0.0f);
		current_state_[2 * i] = Vec3f(start_point.x + scale_x * i, start_point.y + scale_y * i, 0);
		springs_.push_back(Spring(i, i + 1, spring_k, rest_length / (n_ - 1)));
	}
	current_state_[2 * i + 1] = Vec3f(0.0f, 0.0f, 0.0f);
	current_state_[2 * i] = end_point;

}

State PendulumSystem::evalF(const State& state) const {
	const auto drag_k = 0.5f;
	const auto mass = 0.5f;
	auto f = State(2 * n_);
	// YOUR CODE HERE (R4)
	// As in R2, return a derivative of the system state "state".
	f[0] = state[1];
	f[1] = 0;
	Vec3f force;
	Vec3f forceG, forceS1, forceS2, forceD;
	forceG = fGravity(mass);
	for (int i = 1; i != n_ - 1; i++) {
		f[2 * i] = state[2 * i + 1];
		forceD = fDrag(state[2 * i + 1], drag_k);
		forceS1 = fSpring(state[2 * i], state[2 * (i + 1)], springs_[i].k, springs_[i].rlen);
		forceS2 = fSpring(state[2 * i], state[2 * (i - 1)], springs_[i - 1].k, springs_[i - 1].rlen);
		force = forceG + forceS1 + forceS2 + forceD;
		f[2 * i + 1] = force;
	}
	f[2 * (n_ - 1)] = state[2 * (n_ - 1) + 1];
	forceD = fDrag(state[2 * (n_ - 1) + 1], drag_k);
	forceS2 = fSpring(state[2 * (n_ - 1)], state[2 * (n_ - 1) - 2], springs_[n_ - 1].k, springs_[n_ - 1].rlen);
	force = forceG + forceS2 + forceD;
	f[2 * (n_ - 1) + 1] = force;

	return f;
}

#ifdef EIGEN_SPARSECORE_MODULE_H

void PendulumSystem::evalJ(const State& state, SparseMatrix& result, bool initial) const {

	const auto drag_k = 0.5f;
	const auto mass = 0.5f;

	// EXTRA: Evaluate the Jacobian here. Each spring has an effect on four blocks of the matrix -- both of the positions of the endpoints will have an effect on both of the velocities of the endpoints.
}
#endif


Points PendulumSystem::getPoints() {
	auto p = Points(n_);
	for (auto i = 0u; i < n_; ++i) {
		p[i] = current_state_[i * 2];
	}
	return p;
}

Lines PendulumSystem::getLines() {
	auto l = Lines();
	for (const auto& s : springs_) {
		l.push_back(current_state_[2 * s.i1]);
		l.push_back(current_state_[2 * s.i2]);
	}
	return l;
}

void ClothSystem::reset() {
	const auto spring_k = 300.0f;
	const auto width = 1.5f, height = 1.5f; // width and height of the whole grid
	current_state_ = State(2 * x_ * y_);
	// YOUR CODE HERE (R5)
	// Construct a particle system with a x_ * y_ grid of particles,
	// connected with a variety of springs as described in the handout:
	// structural springs, shear springs and flex springs.
	for (int m = 0; m != y_; m++) {
		for (int n = 0; n != x_; n++) {
			int pos = 2 * x_ * m + 2 * n;
			float x = (float)m / (x_ - 1.0f) * height - height / 2.0f;
			float y = (float)n / (y_ - 1.0f) * width * (-1.0f);
			current_state_[pos] = Vec3f(x, 0, y);
		}
	}
	if (springs_.size() == 0) {
		float restLength1 = 1.5f / (x_ - 1.0f);
		float restLength2 = 1.5f / (y_ - 1.0f);
		for (int i = 0; i != x_ * y_; i++) {
			// Structural springs
			// link (i, j) and (i+1, j), except particles on the right border 
			if ((i + 1) % x_ != 0) {
				Spring r(i, i + 1, spring_k, restLength1);
				springs_.push_back(r);
			}
			// link(i, j) and (i, j+1), except particles on the bottom border
			if ((i + 1) <= (y_ - 1) * x_) {
				Spring d(i, i + x_, spring_k, restLength2);
				springs_.push_back(d);
			}
			// Shear springs
			// link(i, j) and (i+1, j+1), except particles on the right and bottom border
			if ((i + 1) % x_ != 0 && (i + 1) <= (y_ - 1) * x_) {
				Spring rd(i, i + x_ + 1, spring_k, FW::sqrt(pow(restLength1, 2) + pow(restLength2, 2)));
				springs_.push_back(rd);
			}
			// link(i, j) and (i+1, j-1), except particles on the right and top border  
			if ((i + 1) % x_ != 0 && (i + 1) > x_) {
				Spring ru(i, i - (x_ - 1), spring_k, FW::sqrt(pow(restLength1, 2) + pow(restLength2, 2)));
				springs_.push_back(ru);
			}
			// Flex springs, except particles on the right two cols and bottom two rows
			if ((i + 1) % x_ != (x_ - 1) && (i + 1) % x_ != 0 && (i + 1) < (y_ - 2) * x_) {
				Spring r2(i, i + 2, spring_k, 2 * restLength1);
				Spring d2(i, i + 2 * x_, spring_k, 2 * restLength2);
				springs_.push_back(r2);
				springs_.push_back(d2);
			}
		}
	}

}

State ClothSystem::evalF(const State& state) const {
	const auto drag_k = 0.08f;
	const auto n = x_ * y_;
	static const auto mass = 0.025f;
	auto f = State(2 * n);
	// YOUR CODE HERE (R5)
	// This will be much like in R2 and R4.
	const auto spring_k = 300.0f;
	float restLength1 = 1.5f / (x_ - 1.0f);
	float restLength2 = 1.5f / (y_ - 1.0f);
	for (int i = 1; i != n; i++) {
		if (i == 380)
			continue;
		f[2 * i] = state[2 * i + 1];
		Vec3f forceG = fGravity(mass);
		Vec3f forceD = fDrag(state[2 * i + 1], drag_k);
		Vec3f forceO(0);
		if ((i + 1) % x_ != 0)
			forceO += fSpring(state[2 * i], state[2 * (i + 1)], spring_k, restLength1);
		if ((i + 1) % x_ != 1)
			forceO += fSpring(state[2 * i], state[2 * (i - 1)], spring_k, restLength1);
		if ((i + x_) < n){
			// Debug
			//cout << i<<", " << i + x_ << "\n";
			forceO += fSpring(state[2 * i], state[2 * (i + x_)], spring_k, restLength2);
		}
		if (i >= x_)
			forceO += fSpring(state[2 * i], state[2 * (i - x_)], spring_k, restLength2);
		if (i % x_ != x_ - 1 && (i + x_ + 1) < n)
			forceO += fSpring(state[2 * i], state[2 * (i + x_ + 1)], spring_k, FW::sqrt(pow(restLength1, 2) + pow(restLength2, 2)));
		if (i % x_ != 0 && i >= x_ + 1)
			forceO += fSpring(state[2 * i], state[2 * (i - x_ - 1)], spring_k, FW::sqrt(pow(restLength1, 2) + pow(restLength2, 2)));
		if (i % x_ != 0 && (i + x_ - 1) < n)
			forceO += fSpring(state[2 * i], state[2 * (i + x_ - 1)], spring_k, FW::sqrt(pow(restLength1, 2) + pow(restLength2, 2)));
		if (i % x_ != x_ - 1 && i >= x_ - 1)
			forceO += fSpring(state[2 * i], state[2 * (i - x_ + 1)], spring_k, FW::sqrt(pow(restLength1, 2) + pow(restLength2, 2)));
		
		if (i % x_ != x_ - 2 && i % x_ != x_ - 1)
			forceO += fSpring(state[2 * i], state[2 * (i + 2)], spring_k, 2 * restLength1);
		if (i % x_ != 1 && i % x_ != 0)
			forceO += fSpring(state[2 * i], state[2 * (i - 2)], spring_k, 2 * restLength1);
		if (i  < n - 2 * x_)
			forceO += fSpring(state[2 * i], state[2 * (i + 2 * x_)], spring_k, 2 * restLength2);
		if (i >= 2 * x_)
			forceO += fSpring(state[2 * i], state[2 * (i - 2 * x_)], spring_k, 2 * restLength2);
		f[2 * i + 1] = (forceG + forceD + forceO) / mass;
	}

	return f;
}

#ifdef EIGEN_SPARSECORE_MODULE_H

void ClothSystem::evalJ(const State& state, SparseMatrix& result, bool initial) const {
	const auto drag_k = 0.08f;
	static const auto mass = 0.025f;

	// EXTRA: Evaluate the Jacobian here. The code is more or less the same as for the pendulum.
}

#endif

Points ClothSystem::getPoints() {
	auto n = x_ * y_;
	auto p = Points(n);
	for (auto i = 0u; i < n; ++i) {
		p[i] = current_state_[2 * i];
	}
	return p;
}

Lines ClothSystem::getLines() {
	auto l = Lines();
	for (const auto& s : springs_) {
		l.push_back(current_state_[2 * s.i1]);
		l.push_back(current_state_[2 * s.i2]);
	}
	return l;
}
State FluidSystem::evalF(const State&) const {
	return State();
}

