/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
* Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
* University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the
* terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the
* License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
* WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with
* ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include <math.h>

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

namespace ORB_SLAM3 {

class LoopClosing;

class Optimizer {
 public:
    void static BundleAdjustment(const std::vector<KeyFrame *> &vpKF,
                                 const std::vector<MapPoint *> &vpMP,
                                 int nIterations = 5,
                                 bool *pbStopFlag = NULL,
                                 const unsigned long nLoopKF = 0,
                                 const bool bRobust = true);

    // LoopClosing 和 Tracking 单目初始化
    void static GlobalBundleAdjustemnt(Map *pMap,
                                       int nIterations = 5,
                                       bool *pbStopFlag = NULL,
                                       const unsigned long nLoopKF = 0,
                                       const bool bRobust = true);
    // LoopClosing 和 LocalMapping IMU初始化
    void static FullInertialBA(Map *pMap,
                               int its,
                               const bool bFixLocal = false,
                               const unsigned long nLoopKF = 0,
                               bool *pbStopFlag = NULL,
                               bool bInit = false,
                               float priorG = 1e2,
                               float priorA = 1e6,
                               Eigen::VectorXd *vSingVal = NULL,
                               bool *bHess = NULL);

    // 好像没有用诶
    void static LocalBundleAdjustment(KeyFrame *pKF,
                                      bool *pbStopFlag,
                                      vector<KeyFrame *> &vpNonEnoughOptKFs);
    
    // LocalMaping : 纯视觉时候调用的 LBA
    void static LocalBundleAdjustment(KeyFrame *pKF,
                                      bool *pbStopFlag,
                                      Map *pMap,
                                      int &num_fixedKF,
                                      int &num_OptKF,
                                      int &num_MPs,
                                      int &num_edges);

    // 好像没有用诶
    void static MergeBundleAdjustmentVisual(KeyFrame *pCurrentKF,
                                            vector<KeyFrame *> vpWeldingKFs,
                                            vector<KeyFrame *> vpFixedKFs,
                                            bool *pbStopFlag);

    // 老版本的单纯只优化姿态: 在 Tracking 线程中,大多在没有IMU或者IMU还没初始化好的时候调用
    int static PoseOptimization(Frame *pFrame);
    //  在 Tracking 线程中, 和上面函数分类讨论调用, IMU初始化好之后可能用
    int static PoseInertialOptimizationLastKeyFrame(Frame *pFrame,
                                                    bool bRecInit = false);
    //  在 Tracking 线程中, 和上面函数分类讨论调用, IMU初始化好之后可能用
    int static PoseInertialOptimizationLastFrame(Frame *pFrame,
                                                 bool bRecInit = false);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise
    // (mono)
    // LoopClosing 中调用
    void static OptimizeEssentialGraph(
        Map *pMap,
        KeyFrame *pLoopKF,
        KeyFrame *pCurKF,
        const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
        const LoopClosing::KeyFrameAndPose &CorrectedSim3,
        const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
        const bool &bFixScale);
    // 好像没有用诶
    void static OptimizeEssentialGraph6DoF(
        KeyFrame *pCurKF,
        vector<KeyFrame *> &vpFixedKFs,
        vector<KeyFrame *> &vpFixedCorrectedKFs,
        vector<KeyFrame *> &vpNonFixedKFs,
        vector<MapPoint *> &vpNonCorrectedMPs,
        double scale);
    // LoopClosing 中调用
    void static OptimizeEssentialGraph(KeyFrame *pCurKF,
                                       vector<KeyFrame *> &vpFixedKFs,
                                       vector<KeyFrame *> &vpFixedCorrectedKFs,
                                       vector<KeyFrame *> &vpNonFixedKFs,
                                       vector<MapPoint *> &vpNonCorrectedMPs);
    // 好像没有用诶
    void static OptimizeEssentialGraph(
        KeyFrame *pCurKF,
        const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
        const LoopClosing::KeyFrameAndPose &CorrectedSim3);
    // LoopClosing 中调用
    // For inetial loopclosing
    void static OptimizeEssentialGraph4DoF(
        Map *pMap,
        KeyFrame *pLoopKF,
        KeyFrame *pCurKF,
        const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
        const LoopClosing::KeyFrameAndPose &CorrectedSim3,
        const map<KeyFrame *, set<KeyFrame *> > &LoopConnections);

    // LoopClosing 中调用
    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    // (OLD)
    static int OptimizeSim3(KeyFrame *pKF1,
                            KeyFrame *pKF2,
                            std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12,
                            const float th2,
                            const bool bFixScale);
    // LoopClosing 中调用
    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    // (NEW)
    static int OptimizeSim3(KeyFrame *pKF1,
                            KeyFrame *pKF2,
                            std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12,
                            const float th2,
                            const bool bFixScale,
                            Eigen::Matrix<double, 7, 7> &mAcumHessian,
                            const bool bAllPoints = false);
    static int OptimizeSim3(KeyFrame *pKF1,
                            KeyFrame *pKF2,
                            vector<MapPoint *> &vpMatches1,
                            vector<KeyFrame *> &vpMatches1KF,
                            g2o::Sim3 &g2oS12,
                            const float th2,
                            const bool bFixScale,
                            Eigen::Matrix<double, 7, 7> &mAcumHessian,
                            const bool bAllPoints = false);

    // For inertial systems
    // 在LocalMapping 线程中调用做 LBA, 和 LocalBundleAdjustment 对应
    void static LocalInertialBA(KeyFrame *pKF,
                                bool *pbStopFlag,
                                Map *pMap,
                                int &num_fixedKF,
                                int &num_OptKF,
                                int &num_MPs,
                                int &num_edges,
                                bool bLarge = false,
                                bool bRecInit = false);

    // LoopClosing 中调用
    void static MergeInertialBA(KeyFrame *pCurrKF,
                                KeyFrame *pMergeKF,
                                bool *pbStopFlag,
                                Map *pMap,
                                LoopClosing::KeyFrameAndPose &corrPoses);

    // 在LoopClosing 中调用的
    // Local BA in welding area when two maps are merged
    void static LocalBundleAdjustment(KeyFrame *pMainKF,
                                      vector<KeyFrame *> vpAdjustKF,
                                      vector<KeyFrame *> vpFixedKF,
                                      bool *pbStopFlag);

    // Marginalize block element (start:end,start:end). Perform Schur
    // complement.
    // Marginalized elements are filled with zeros.
    static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd &H,
                                       const int &start,
                                       const int &end);
    // 没调用??
    // Condition block element (start:end,start:end). Fill with zeros.
    static Eigen::MatrixXd Condition(const Eigen::MatrixXd &H,
                                     const int &start,
                                     const int &end);
    // 好像没有用诶
    // Remove link between element 1 and 2. Given elements 1,2 and 3 must define
    // the whole matrix.
    static Eigen::MatrixXd Sparsify(const Eigen::MatrixXd &H,
                                    const int &start1,
                                    const int &end1,
                                    const int &start2,
                                    const int &end2);
    // 都只用在了初始化时候
    // Inertial pose-graph
    void static InertialOptimization(Map *pMap,
                                     Eigen::Matrix3d &Rwg,
                                     double &scale,
                                     Eigen::Vector3d &bg,
                                     Eigen::Vector3d &ba,
                                     bool bMono,
                                     Eigen::MatrixXd &covInertial,
                                     bool bFixedVel = false,
                                     bool bGauss = false,
                                     float priorG = 1e2,
                                     float priorA = 1e6);
    void static InertialOptimization(Map *pMap,
                                     Eigen::Vector3d &bg,
                                     Eigen::Vector3d &ba,
                                     float priorG = 1e2,
                                     float priorA = 1e6);
    void static InertialOptimization(vector<KeyFrame *> vpKFs,
                                     Eigen::Vector3d &bg,
                                     Eigen::Vector3d &ba,
                                     float priorG = 1e2,
                                     float priorA = 1e6);
    void static InertialOptimization(Map *pMap,
                                     Eigen::Matrix3d &Rwg,
                                     double &scale);
};

}  // namespace ORB_SLAM3

#endif  // OPTIMIZER_H
