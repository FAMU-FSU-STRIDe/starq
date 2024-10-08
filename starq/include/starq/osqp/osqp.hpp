#ifndef STARQ_OSQP__OSQP_HPP_
#define STARQ_OSQP__OSQP_HPP_

#include "starq/mpc/mpc_solver.hpp"

#include "osqp/osqp.h"

namespace starq::osqp
{

    /// @brief OSQP class
    class OSQP : public mpc::MPCSolver
    {
    public:
        using Ptr = std::shared_ptr<OSQP>;

        /// @brief Create a new OSQP object
        OSQP();

        /// @brief Destroy the OSQP object
        ~OSQP();

        /// @brief Update the QP solver
        /// @param config The MPC configuration
        /// @return If the update was successful
        bool update(mpc::MPCConfiguration::Ptr config) override;

        /// @brief Solve the QP problem
        /// @return If the problem was solved successfully
        bool solve() override;

        /// @brief Get the QP problem
        /// @return The QP problem
        starq::mpc::QPProblem::Ptr getQPProblem() const;

        /// @brief Get the solution
        /// @return The solution
        starq::mpc::MPCSolution::Ptr getSolution() const override;

        /// @brief Get the first force state of the solution
        /// @return The first force state
        starq::mpc::FootForceState getFirstForceState() const override;

        /// @brief Get the OSQP settings
        /// @return The OSQP settings
        OSQPSettings *getSettings() const { return settings_; }

        /// @brief Get the OSQP solver
        /// @return The OSQP solver
        OSQPSolver *getSolver() const { return solver_; }

    private:
        void convertEigenSparseToCSC(const Eigen::SparseMatrix<Float> &matrix,
                                     OSQPCscMatrix *&M, OSQPInt &Mnnz, OSQPFloat *&Mx, OSQPInt *&Mi, OSQPInt *&Mp);


        void saveMPCSolution();

        mpc::QPProblem::Ptr qp_problem_ = nullptr;

        mpc::MPCSolution::Ptr solution_ = nullptr;

        OSQPInt n_, m_;

        OSQPSolver *solver_ = nullptr;
        OSQPSettings *settings_ = nullptr;

        OSQPFloat *q_ = nullptr;
        OSQPFloat *l_ = nullptr;
        OSQPFloat *u_ = nullptr;

        OSQPCscMatrix *P_ = nullptr;
        OSQPInt Pnnz_;
        OSQPFloat *Px_ = nullptr;
        OSQPInt *Pi_ = nullptr;
        OSQPInt *Pp_ = nullptr;

        OSQPCscMatrix *A_ = nullptr;
        OSQPInt Annz_;
        OSQPFloat *Ax_ = nullptr;
        OSQPInt *Ai_ = nullptr;
        OSQPInt *Ap_ = nullptr;
    };

}

#endif