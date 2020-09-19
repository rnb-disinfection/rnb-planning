/*
 * ExternalInterpolator.h
 *
 *  Created on: Aug 7, 2018
 *      Author: Jonghoon Park, Hanter Jung
 */

#ifndef EXTERNALINTERPOLATOR_H_
#define EXTERNALINTERPOLATOR_H_

#include <stdio.h>
#include <string.h>

#include <iostream> //FIXME for test

#include "LieGroup/LieGroup.h"
#include "Framework/CompositeHTransformTask.h"
#include "Framework/ExtendedTask.h"

#include "InterpolationData.h"



#include "online_interpolator.h"


namespace NRMKFoundation {
namespace internal {
}

template<int DIM>
class ExternalJointInterpolator {
  public:
    enum {
        MAX_TRAJ_LENGTH = NRMKMotion::AbstractInterpolationData::MAX_TRAJ_LENGTH,
        JOINT_VEC_SIZE = DIM
    };

    typedef Eigen::Matrix<double, DIM, 1> JointVec;

    typedef NRMKMotion::JointInterpolationData<DIM> JointInterpolationData;

  public:
    OnlineInterpolator<DIM> online_interpolator;
    bool _flag_online;

    ExternalJointInterpolator()
            : _dim(DIM),
              _dt(0),
              _isTargetReached(false),
              _isTrajSet(false),
              _isTrajSetFailed(false),
              _flag_online(false) {
    }

    //FIXME for test...
    inline void setTraj(const double &t0, const char *path) {
        int size;

        std::ifstream dat(path);

        dat >> size;
        _length = size;
        _t0 = t0;

        printf("Total Length : %d\n", _length);

        for (int i = 0; i < size; i++) {
            for (int j = 0; j < DIM; j++) dat >> _qd[i][j];
            for (int j = 0; j < DIM; j++) dat >> _qdotd[i][j];
            for (int j = 0; j < DIM; j++) dat >> _qddotd[i][j];
        }
    }
    inline void setTraj(const double &t0, FILE *path) {
        _flag_online = false;
        int size;
        fscanf(path, "%d", &size);
        _length = size;
        _t0 = t0;

        for (int i = 0; i < size; i++) {
            fscanf(path, "%lf %lf %lf %lf %lf %lf", _qd[i][0], _qd[i][1], _qd[i][2], _qd[i][3], _qd[i][4], _qd[i][5]);
            fscanf(path,
                   "%lf %lf %lf %lf %lf %lf",
                   _qdotd[i][0],
                   _qdotd[i][1],
                   _qdotd[i][2],
                   _qdotd[i][3],
                   _qdotd[i][4],
                   _qdotd[i][5]);
            fscanf(path,
                   "%lf %lf %lf %lf %lf %lf",
                   _qddotd[i][0],
                   _qddotd[i][1],
                   _qddotd[i][2],
                   _qddotd[i][3],
                   _qddotd[i][4],
                   _qddotd[i][5]);
        }
    }

    inline void setTraj(const double &t0, const NRMKMotion::JointInterpolationData<DIM> *intprData) {
        _flag_online = false;
        _isTrajSet = false;
        _isTargetReached = false;

        _length = intprData->getTrajLen();
        if (_length == 0 || _dim != intprData->getDIM() || _dt != intprData->getPeriod()) {
            _idx = 0;
            _t0 = 0;
            _length = 0;
            _isTargetReached = true;    //FIXME
            _isTrajSetFailed = true;
            return;
        }

        memcpy(_qd, intprData->qd(), sizeof(double) * DIM * _length);
        memcpy(_qdotd, intprData->qdotd(), sizeof(double) * DIM * _length);
        memcpy(_qddotd, intprData->qddotd(), sizeof(double) * DIM * _length);

        if (_length == 1) // if _length==1, online trajectory
        {
            _flag_online = true;
            online_interpolator.init_thread(intprData->getPeriod(), online_interpolator.period_s, _qd[0]);
        }

        _idx = 0;
        _t0 = t0;
        _isTrajSet = true;
        _isTrajSetFailed = false;
    }

    inline void traj(double t, JointVec &pd, JointVec &vd, JointVec &ad) {
        if (_length == 0 || _isTrajSetFailed) {
            pd << 0, 0, 0, 0, 0, 0;
            vd << 0, 0, 0, 0, 0, 0;
            ad << 0, 0, 0, 0, 0, 0;
            return;
        }

        int idx = (t - _t0) / _dt;
        if (_flag_online)    //target reached
        {
            _idx = idx;
            _isTargetReached = false;

            online_interpolator.get_next_qc(pd, vd, ad);
        } else if (idx > _length - 1)    //target reached
        {
            idx = _length - 1;
            _idx = idx;
            _isTargetReached = true;
            _isTrajSet = false;

            if (DIM == 3) {
                pd << _qd[idx][0], _qd[idx][1], _qd[idx][2];
                vd << 0, 0, 0;
                ad << 0, 0, 0;
            } else if (DIM == 6) {
                pd << _qd[idx][0], _qd[idx][1], _qd[idx][2], _qd[idx][3], _qd[idx][4], _qd[idx][5];
                vd << 0, 0, 0, 0, 0, 0;
                ad << 0, 0, 0, 0, 0, 0;
            } else if (DIM == 7) {
                pd << _qd[idx][0], _qd[idx][1], _qd[idx][2], _qd[idx][3], _qd[idx][4], _qd[idx][5], _qd[idx][6];
                vd << 0, 0, 0, 0, 0, 0, 0;
                ad << 0, 0, 0, 0, 0, 0, 0;
            }
        } else if (idx < 0)    //ready
        {
            //FIXME prev time (will be fixed by Park daeri)
            idx = 0;
            _idx = idx;
            _isTargetReached = false;

            if (DIM == 3) {
                pd << _qd[idx][0], _qd[idx][1], _qd[idx][2];
                vd << 0, 0, 0;
                ad << 0, 0, 0;
            } else if (DIM == 6) {
                pd << _qd[idx][0], _qd[idx][1], _qd[idx][2], _qd[idx][3], _qd[idx][4], _qd[idx][5];
                vd << 0, 0, 0, 0, 0, 0;
                ad << 0, 0, 0, 0, 0, 0;
            } else if (DIM == 7) {
                pd << _qd[idx][0], _qd[idx][1], _qd[idx][2], _qd[idx][3], _qd[idx][4], _qd[idx][5], _qd[idx][6];
                vd << 0, 0, 0, 0, 0, 0, 0;
                ad << 0, 0, 0, 0, 0, 0, 0;
            }
        } else {
            _idx = idx;
            _isTargetReached = false;

            if (DIM == 3) {
                pd << _qd[idx][0], _qd[idx][1], _qd[idx][2];
                vd << _qdotd[idx][0], _qdotd[idx][1], _qdotd[idx][2];
                ad << _qddotd[idx][0], _qddotd[idx][1], _qddotd[idx][2];
            } else if (DIM == 6) {
                pd << _qd[idx][0], _qd[idx][1], _qd[idx][2], _qd[idx][3], _qd[idx][4], _qd[idx][5];
                vd << _qdotd[idx][0], _qdotd[idx][1], _qdotd[idx][2], _qdotd[idx][3], _qdotd[idx][4], _qdotd[idx][5];
                ad
                        << _qddotd[idx][0], _qddotd[idx][1], _qddotd[idx][2], _qddotd[idx][3], _qddotd[idx][4], _qddotd[idx][5];
            } else if (DIM == 7) {
                pd << _qd[idx][0], _qd[idx][1], _qd[idx][2], _qd[idx][3], _qd[idx][4], _qd[idx][5], _qd[idx][6];
                vd
                        << _qdotd[idx][0], _qdotd[idx][1], _qdotd[idx][2], _qdotd[idx][3], _qdotd[idx][4], _qdotd[idx][5], _qdotd[idx][6];
                ad
                        << _qddotd[idx][0], _qddotd[idx][1], _qddotd[idx][2], _qddotd[idx][3], _qddotd[idx][4], _qddotd[idx][5], _qddotd[idx][6];
            }
        }
    }

    inline void setPeriod(double dt) { _dt = dt; }
    inline void setTrajStartTime(double t0) { _t0 = t0; }
    inline int getTrajIndex() { return _idx; }
    inline bool isTargetReached() { return _isTargetReached; }
    inline bool isTrajSetFailed() { return _isTrajSetFailed; }

  protected:
    //  ---------------------- Doxygen info ----------------------
    //! \brief DIM
    //  ----------------------------------------------------------
    const int _dim;

    //  ---------------------- Doxygen info ----------------------
    //! \brief Time period
    //  ----------------------------------------------------------
    double _dt;

    //  ---------------------- Doxygen info ----------------------
    //! \brief Path Length
    //  ----------------------------------------------------------
    int _length;

    //  ---------------------- Doxygen info ----------------------
    //! \brief Initial time
    //  ----------------------------------------------------------
    double _t0;

    //  ---------------------- Doxygen info ----------------------
    //! \brief current traj index
    //  ----------------------------------------------------------
    int _idx;

    //  ---------------------- Doxygen info ----------------------
    //! \brief Time
    //  ----------------------------------------------------------
    double _t[MAX_TRAJ_LENGTH];

    //  ---------------------- Doxygen info ----------------------
    //! \brief Desired Position
    //  ----------------------------------------------------------
    double _qd[MAX_TRAJ_LENGTH][DIM];

    //  ---------------------- Doxygen info ----------------------
    //! \brief Desired Velocity
    //  ----------------------------------------------------------
    double _qdotd[MAX_TRAJ_LENGTH][DIM];

    //  ---------------------- Doxygen info ----------------------
    //! \brief Desired Acceleration
    //  ----------------------------------------------------------
    double _qddotd[MAX_TRAJ_LENGTH][DIM];

    //  ---------------------- Doxygen info ----------------------
    //! \brief isTargetReached
    //  ----------------------------------------------------------
    bool _isTargetReached;
    bool _isTrajSet;
    bool _isTrajSetFailed;
};

class ExternalTaskDisplacementInterpolator : public ExternalJointInterpolator<3> {
  public:
    ExternalTaskDisplacementInterpolator()
            : ExternalJointInterpolator<3>() {}

    inline void setTraj(const double &t0, const NRMKMotion::TaskInterpolationData *intprData) {
        _isTrajSet = false;
        _isTargetReached = false;

        _length = intprData->getTrajLen();
        if (_length == 0 || _dt != intprData->getPeriod()) {
            _idx = 0;
            _t0 = 0;
            _length = 0;
            _isTargetReached = true;    //FIXME
            _isTrajSetFailed = true;
            return;
        }

        NRMKMotion::TaskInterpolationData::IntprValTypeConst pd = intprData->pd();
        NRMKMotion::TaskInterpolationData::IntprValTypeConst pdotd = intprData->pdotd();
        NRMKMotion::TaskInterpolationData::IntprValTypeConst pddotd = intprData->pddotd();

        for (int idx = 0; idx < _length; idx++) {
            memcpy(&_qd[idx][0], &pd[idx][0]/*0~2*/, sizeof(double) * 3);
            memcpy(&_qdotd[idx][0], &pdotd[idx][0], sizeof(double) * 3);
            memcpy(&_qddotd[idx][0], &pddotd[idx][0], sizeof(double) * 3);
        }

        _idx = 0;
        _t0 = t0;
        _isTrajSet = true;
        _isTrajSetFailed = false;
    }
};

class ExternalTaskRotationInterpolator {
  public:
    enum {
        MAX_TRAJ_LENGTH = NRMKMotion::AbstractInterpolationData::MAX_TRAJ_LENGTH,
        TASK_VEC_SIZE = 6,
        ROT_VEC_SIZE = 3
    };

    typedef LieGroup::Rotation PosType; //!< Typedef of the type for position variable
    typedef LieGroup::Vector3D VelType; //!< Typedef of the type for velocity variable
    typedef LieGroup::Vector3D AccType; //!< Typedef of the type for acceleration variable

    typedef NRMKMotion::TaskInterpolationData TaskInterpolationData;

  public:

  public:
    ExternalTaskRotationInterpolator()
            : _dt(0), _isTargetReached(false), _isTrajSet(false), _isTrajSetFailed(false) {
    }

    //FIXME for test...
    inline void setTraj(double t, double pd[], double pdotd[], double pddotd[], int idx) {
        _t[idx] = t;
        for (int i = 0; i < 3; i++) {
            _pd[idx][i] = pd[3 + i];
            _pdotd[idx][i] = pdotd[3 + i];
            _pddotd[idx][i] = pddotd[3 + i];
        }

        if (idx == 0) {
            _R0 = LieGroup::Rotation(LieGroup::Vector3D(pd[3], pd[4], pd[5]), 2, 1, 0);
            _t0 = t;
        }
    }

    inline void setTraj(const double &t0, const NRMKMotion::TaskInterpolationData *intprData) {
        _isTrajSet = false;
        _isTargetReached = false;

        _length = intprData->getTrajLen();
        if (_length == 0 || _dt != intprData->getPeriod()) {
            _idx = 0;
            _t0 = 0;
            _length = 0;
            _isTargetReached = true;    //FIXME
            _isTrajSetFailed = true;

            printf("Rotation Length : %d\n", _length);
            printf("Rotation Period : %f, %f\n", _dt, intprData->getPeriod());

            return;
        }

        const NRMKMotion::TaskInterpolationData::IntprValTypeConst pd = intprData->pd();
        const NRMKMotion::TaskInterpolationData::IntprValTypeConst pdotd = intprData->pdotd();
        const NRMKMotion::TaskInterpolationData::IntprValTypeConst pddotd = intprData->pddotd();

        //prev ver (xyzuvw all)
//		memcpy(_pd, 	intprData.pd(), 	sizeof(double)*TASK_VEC_SIZE*_length);
//		memcpy(_pdotd, 	intprData.pdotd(), 	sizeof(double)*TASK_VEC_SIZE*_length);
//		memcpy(_pddotd, intprData.pddotd(), sizeof(double)*TASK_VEC_SIZE*_length);

        //only uvw rot
        for (int idx = 0; idx < _length; idx++) {
            memcpy(&_pd[idx][0], &pd[idx][3]/*3~5*/, sizeof(double) * 3);
            memcpy(&_pdotd[idx][0], &pdotd[idx][3], sizeof(double) * 3);
            memcpy(&_pddotd[idx][0], &pddotd[idx][3], sizeof(double) * 3);


            /// TODO : Optimization Required (reverse is necessary about uvw)

            double tmp;

            tmp = _pd[idx][0];
            _pd[idx][0] = _pd[idx][2];
            _pd[idx][2] = tmp;
            tmp = _pdotd[idx][0];
            _pdotd[idx][0] = _pdotd[idx][2];
            _pdotd[idx][2] = tmp;
            tmp = _pddotd[idx][0];
            _pddotd[idx][0] = _pddotd[idx][2];
            _pddotd[idx][2] = tmp;

            ///
        }

        _R0 = LieGroup::Rotation(LieGroup::Vector3D(_pd[0][0], _pd[0][1], _pd[0][2]), 2, 1, 0);
        _idx = 0;
        _t0 = t0;
        _isTrajSet = true;
        _isTrajSetFailed = false;

//		printf("R0 : \n %12.8f, %12.8f, %12.8f\n%12.8f, %12.8f, %12.8f\n%12.8f, %12.8f, %12.8f\n", _R0(0,0));
    }

    inline void traj(double t, PosType &pos, VelType &vel, AccType &acc) {
        int idx = (t - _t0) / _dt;

        if (idx > _length - 1)    //target reached
        {
            idx = _length - 1;
            _idx = idx;
            _isTargetReached = true;
            _isTrajSet = false;
        } else if (idx < 0)    //ready
        {
            //FIXME prev time (will be fixed by Park daeri)
            idx = 0;
            _idx = idx;
            _isTargetReached = false;
        } else {
            _idx = idx;
            _isTargetReached = false;
        }

        pos = LieGroup::Rotation(LieGroup::Vector3D(_pd[idx][0], _pd[idx][1], _pd[idx][2]), 2, 1, 0);
        for (int i = 0; i < 3; i++) {
            vel[i] = _pdotd[idx][i];
            acc[i] = _pddotd[idx][i];
        }
    }

    inline void setPeriod(double dt) { _dt = dt; }
    inline void setTrajStartTime(double t0) { _t0 = t0; }
    inline int getTrajIndex() { return _idx; }
    inline bool isTargetReached() { return _isTargetReached; }
    inline bool isTrajSetFailed() { return _isTrajSetFailed; }

  protected:
    //  ---------------------- Doxygen info ----------------------
    //! \brief Time period
    //  ----------------------------------------------------------
    double _dt;

    //  ---------------------- Doxygen info ----------------------
    //! \brief Path Length
    //  ----------------------------------------------------------
    int _length;

    //  ---------------------- Doxygen info ----------------------
    //! \brief Initial time
    //  ----------------------------------------------------------
    double _t0;

    //  ---------------------- Doxygen info ----------------------
    //! \brief current traj index
    //  ----------------------------------------------------------
    int _idx;

    //  ---------------------- Doxygen info ----------------------
    //! \brief Time
    //  ----------------------------------------------------------
    double _t[MAX_TRAJ_LENGTH];

    //  ---------------------- Doxygen info ----------------------
    //! \brief Desired Position
    //  ----------------------------------------------------------
    double _pd[MAX_TRAJ_LENGTH][ROT_VEC_SIZE];

    //  ---------------------- Doxygen info ----------------------
    //! \brief Desired Velocity
    //  ----------------------------------------------------------
    double _pdotd[MAX_TRAJ_LENGTH][ROT_VEC_SIZE];

    //  ---------------------- Doxygen info ----------------------
    //! \brief Desired Acceleration
    //  ----------------------------------------------------------
    double _pddotd[MAX_TRAJ_LENGTH][ROT_VEC_SIZE];

    //  ---------------------- Doxygen info ----------------------
    //! \brief Initial Rotation
    //  ----------------------------------------------------------
    LieGroup::Rotation _R0;

    //  ---------------------- Doxygen info ----------------------
    //! \brief isTargetReached
    //  ----------------------------------------------------------
    bool _isTargetReached;
    bool _isTrajSet;
    bool _isTrajSetFailed;
};

template<typename TaskKinematics, typename DispInterpolatorType = ExternalTaskDisplacementInterpolator, typename RotInterpolatorType = ExternalTaskRotationInterpolator>
class CompositeHTransformExternalTaskInterpolator {
  public:
    enum {
        MAX_TRAJ_LENGTH = NRMKMotion::AbstractInterpolationData::MAX_TRAJ_LENGTH,
        TASK_VEC_SIZE = 6,
        DISP_VEC_SIZE = DispInterpolatorType::JOINT_VEC_SIZE,
        ROT_VEC_SIZE = RotInterpolatorType::ROT_VEC_SIZE
    };
  public:
    typedef NRMKFoundation::CompositeHTransformTaskPosition PosType;
    typedef NRMKFoundation::CompositeHTransformTaskVelocity VelType;
    typedef VelType AccType;

    typedef NRMKFoundation::ExtendedTaskPosition<TaskKinematics> ExtPosType;
    typedef NRMKFoundation::ExtendedTaskVelocity<TaskKinematics> ExtVelType;
    typedef NRMKFoundation::ExtendedTaskAcceleration<TaskKinematics> ExtAccType;

  public:
    CompositeHTransformExternalTaskInterpolator() {
    }

    //FIXME for test...
    /*inline void setTraj(double t, FILE* path)
    {
        double pDisp[3];
        double pRot[3];
        double pdotDisp[3];
        double pdotRot[3];
        double pddotDisp[3];
        double pddotRot[3];

        int length;

        fscanf(path, "%d", &length);

        _rotIntpr.setLength(length);
        _dispIntpr.setLength(length);

        for(int i = 0 ; i < length ; i++)
        {
            // Order : x, y, z, u, v, w
            fscanf(path, "%lf %lf %lf %lf %lf %lf", &pDisp[0], &pDisp[1], &pDisp[2], &pRot[0], &pRot[1], &pRot[2]);
            fscanf(path, "%lf %lf %lf %lf %lf %lf", &pdotDisp[0], &pdotDisp[1], &pdotDisp[2], &pdotRot[0], &pdotRot[1], &pdotRot[2]);
            fscanf(path, "%lf %lf %lf %lf %lf %lf", &pddotDisp[0], &pddotDisp[1], &pddotDisp[2], &pddotRot[0], &pddotRot[1], &pddotRot[2]);

            _rotIntpr.setTraj(t, pRot, pdotRot, pddotRot, i);
            _dispIntpr.setTraj(t, pDisp, pdotDisp, pddotDisp, i);
        }
    }*/

    inline void setTraj(const double &t0, const NRMKMotion::TaskInterpolationData *intprData) {
        _rotIntpr.setTraj(t0, intprData);
        _dispIntpr.setTraj(t0, intprData);
    }

    inline void traj(double t, PosType &des_pos, VelType &des_vel, AccType &des_acc) {
        _rotIntpr.traj(t, des_pos.rot, des_vel.rot, des_acc.rot);
        _dispIntpr.traj(t, des_pos.disp, des_vel.disp, des_acc.disp);
    }

    inline void traj(double t, ExtPosType &des_pos, ExtVelType &des_vel, ExtAccType &des_acc) {
        traj(t, des_pos.task, des_vel.task, des_acc.task);
    }

    inline void setPeriod(double dt) {
        _rotIntpr.setPeriod(dt);
        _dispIntpr.setPeriod(dt);
    }

    inline void setTrajStartTime(double t0) {
        _rotIntpr.setTrajStartTime(t0);
        _dispIntpr.setTrajStartTime(t0);
    }
    inline int getTrajIndex() {
        return _rotIntpr.getTrajIndex();
//		return _dispIntpr.getTrajIndex();
    }

    inline bool isTargetReached() { return (_rotIntpr.isTargetReached() && _dispIntpr.isTargetReached()); }
    inline bool isTrajSetFailed() { return (_rotIntpr.isTrajSetFailed() || _dispIntpr.isTrajSetFailed()); }

  private:
    RotInterpolatorType _rotIntpr;
    DispInterpolatorType _dispIntpr;
};

}    /* namespace NRMKFoundation */

#endif /* EXTERNALINTERPOLATOR_H_ */
