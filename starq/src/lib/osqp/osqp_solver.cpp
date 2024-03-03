#include "starq/osqp/osqp_solver.hpp"

#include <iostream>

namespace starq::osqp
{

    OSQP_MPCSolver::OSQP_MPCSolver(const MPCConfiguration &config)
        : MPCSolver(config)
    {
        osqp_.settings = new OSQPSettings;
        osqp_set_default_settings(osqp_.settings);
    }

    OSQP_MPCSolver::~OSQP_MPCSolver()
    {
        osqp_cleanup(osqp_.solver);
        delete osqp_.P;
        delete osqp_.A;
        delete osqp_.settings;
    }

    bool OSQP_MPCSolver::setup()
    {
        setupQP();
        setupOSQP();
        return true;
    }

    MPCSolution OSQP_MPCSolver::solve()
    {
        MPCSolution solution;
        solution.exit_flag = osqp_solve(osqp_.solver);
        solution.run_time = microseconds(static_cast<int>(osqp_.solver->info->run_time * 1e6));
        solution.setup_time = microseconds(static_cast<int>(osqp_.solver->info->setup_time * 1e6));
        solution.solve_time = microseconds(static_cast<int>(osqp_.solver->info->solve_time * 1e6));

        auto x = osqp_.solver->solution->x;
        for (size_t i = 0; i < config_.window_size; i++)
        {
            const int ox = 13 * i;
            Vector3f orientation(x[ox], x[ox + 1], x[ox + 2]);
            Vector3f position(x[ox + 3], x[ox + 4], x[ox + 5]);
            Vector3f angular_velocity(x[ox + 6], x[ox + 7], x[ox + 8]);
            Vector3f linear_velocity(x[ox + 9], x[ox + 10], x[ox + 11]);
            CenterOfMassState state = {position, orientation, linear_velocity, angular_velocity};
            solution.x_star.push_back(state);
        }

        int offset = nx_;
        for (size_t i = 0; i < config_.window_size - 1; i++)
        {
            FootForceState forces;
            for (int j = 0; j < n_legs_[i]; j++)
            {
                if (config_.stance_trajectory[i][j])
                {
                    Vector3f force(x[offset], x[offset + 1], x[offset + 2]);
                    forces.push_back(std::make_pair(true, force));
                    offset += 3;
                }
                else
                {
                    forces.push_back(std::make_pair(false, Vector3f::Zero()));
                }
            }
            solution.u_star.push_back(forces);
        }

        return solution;
    }

    void OSQP_MPCSolver::setupQP()
    {
        const int n = nx_ + nu_;
        const int m = nx_ + 2 * nu_;

        H_.resize(n, n);
        q_.resize(n);
        Ac_.resize(m, n);
        lc_.resize(m);
        uc_.resize(m);

        calculateQPHessian();
        calculateQPGradient();
        calculateQPLinearConstraint();
        calculateQPLowerBound();
        calculateQPUpperBound();
    }

    void OSQP_MPCSolver::calculateQPHessian()
    {
        const int Nn = config_.window_size;
        const int Nx = 13;
        std::vector<Triplet<double>> triplets;

        for (int i = 0; i < Nn; i++)
        {
            const int idx = Nx * i;
            for (int j = 0; j < Nx; j++)
                for (int k = 0; k < Nx; k++)
                {
                    const double value = Q_[i](j, k);
                    if (value != 0)
                        triplets.push_back(Triplet<double>(idx + j, idx + k, value));
                }
        }

        int idx = Nx * Nn;
        for (int i = 0; i < Nn - 1; i++)
        {
            const int Nu = 3 * n_legs_[i];
            for (int j = 0; j < Nu; j++)
                for (int k = 0; k < Nu; k++)
                {
                    const double value = R_[i](j, k);
                    if (value != 0)
                        triplets.push_back(Triplet<double>(idx + j, idx + k, value));
                }
            idx += Nu;
        }
        H_.setFromTriplets(triplets.begin(), triplets.end());
    }

    void OSQP_MPCSolver::calculateQPGradient()
    {
        const int Nn = config_.window_size;
        const int Nx = 13;

        for (int i = 0; i < Nn; i++)
        {
            const int xi = i * Nx;
            const VectorXd Gx = Q_[i].cast<double>() * -xref_[i].cast<double>();
            q_.block(xi, 0, Nx, 1) = Gx;
        }

        int idx = Nx * Nn;
        for (int i = 0; i < Nn - 1; i++)
        {
            const int Nu = 3 * n_legs_[i];
            const VectorXd Gu = VectorXd::Zero(Nu);
            q_.block(idx, 0, Nu, 1) = Gu;
            idx += Nu;
        }
    }

    void OSQP_MPCSolver::calculateQPLinearConstraint()
    {
        const int Nn = config_.window_size;
        const int Nx = 13;

        std::vector<Triplet<double>> triplets;
        for (int i = 0; i < Nx * Nn; i++)
            triplets.push_back(Triplet<double>(i, i, -1));

        for (int i = 0; i < Nn - 1; i++)
            for (int j = 0; j < Nx; j++)
                for (int k = 0; k < Nx; k++)
                {
                    const double value = A_[i](j, k);
                    if (value != 0)
                        triplets.push_back(Triplet<double>(
                            Nx * (i + 1) + j,
                            Nx * i + k,
                            value));
                }

        int idx = Nx * Nn;
        for (int i = 0; i < Nn - 1; i++)
        {
            const int Nu = 3 * n_legs_[i];
            for (int j = 0; j < Nx; j++)
                for (int k = 0; k < Nu; k++)
                {
                    const double value = B_[i](j, k);
                    if (value != 0)
                        triplets.push_back(Triplet<double>(
                            Nx * (i + 1) + j,
                            idx + k,
                            value));
                }
            idx += Nu;
        }

        idx = 0;
        for (int i = 0; i < Nn - 1; i++)
        {
            const int Nu = 3 * n_legs_[i];
            for (int j = 0; j < 2 * Nu; j++)
                for (int k = 0; k < Nu; k++)
                {
                    const double value = C_[i](j, k);
                    if (value != 0)
                        triplets.push_back(Triplet<double>(
                            nx_ + 2 * idx + j,
                            nx_ + idx + k,
                            value));
                }
            idx += Nu;
        }

        Ac_.setFromTriplets(triplets.begin(), triplets.end());
    }

    void OSQP_MPCSolver::calculateQPLowerBound()
    {
        const int Nn = config_.window_size;
        const int Nx = 13;
        const int Neq = nx_;
        const int Nineq = 2 * nu_;

        lc_.block(0, 0, Neq, 1) = VectorXd::Zero(Neq);
        lc_.block(0, 0, Nx, 1) = -xref_[0].cast<double>();

        lc_.block(Neq, 0, Nineq, 1) = VectorXd::Zero(Nineq);
        int idx = Neq;
        for (int i = 0; i < Nn - 1; i++)
        {
            const int Nuc = 6 * n_legs_[i];
            lc_.block(idx, 0, Nuc, 1) = cl_[i].cast<double>();
            idx += Nuc;
        }
    }

    void OSQP_MPCSolver::calculateQPUpperBound()
    {
        const int Nn = config_.window_size;
        const int Nx = 13;
        const int Neq = nx_;
        const int Nineq = 2 * nu_;

        uc_.block(0, 0, Neq, 1) = VectorXd::Zero(Neq);
        uc_.block(0, 0, Nx, 1) = -xref_[0].cast<double>();

        uc_.block(Neq, 0, Nineq, 1) = VectorXd::Zero(Nineq);
        int idx = Neq;
        for (int i = 0; i < Nn - 1; i++)
        {
            const int Nuc = 6 * n_legs_[i];
            uc_.block(idx, 0, Nuc, 1) = cu_[i].cast<double>();
            idx += Nuc;
        }
    }

    void OSQP_MPCSolver::setupOSQP()
    {
        osqp_.n = nx_ + nu_;
        osqp_.m = nx_ + 2 * nu_;

        osqp_.q = q_.data();
        osqp_.l = lc_.data();
        osqp_.u = uc_.data();

        convertEigenSparseToCSC(H_, osqp_.P, osqp_.Pnnz, osqp_.Px, osqp_.Pi, osqp_.Pp);
        convertEigenSparseToCSC(Ac_, osqp_.A, osqp_.Annz, osqp_.Ax, osqp_.Ai, osqp_.Ap);

        osqp_setup(&osqp_.solver, osqp_.P, osqp_.q, osqp_.A, osqp_.l, osqp_.u, osqp_.m, osqp_.n, osqp_.settings);
    }

    void OSQP_MPCSolver::convertEigenSparseToCSC(const SparseMatrix<double> &matrix,
                                                 OSQPCscMatrix *&M, OSQPInt &Mnnz, OSQPFloat *&Mx, OSQPInt *&Mi, OSQPInt *&Mp)
    {
        M = new OSQPCscMatrix;
        Mnnz = matrix.nonZeros();
        Mx = new OSQPFloat[Mnnz];
        Mi = new OSQPInt[Mnnz];
        Mp = new OSQPInt[matrix.cols() + 1];

        int k = 0;
        Mp[0] = 0;
        for (int j = 0; j < matrix.outerSize(); ++j)
        {
            for (SparseMatrix<double>::InnerIterator it(matrix, j); it; ++it)
            {
                Mx[k] = it.value();
                Mi[k] = it.row();
                ++k;
            }
            Mp[j + 1] = k;
        }
        csc_set_data(M, matrix.rows(), matrix.cols(), Mnnz, Mx, Mi, Mp);
    }

}