
void Optimizer::LocalInertialBA(KeyFrame* pKF,
                                bool* pbStopFlag,
                                Map* pMap,
                                int& num_fixedKF,
                                int& num_OptKF,
                                int& num_MPs,
                                int& num_edges,
                                bool bLarge,
                                bool bRecInit) {
    Map* pCurrentMap = pKF->GetMap();

    int maxOpt = 10;
    int opt_it = 10;
    if (bLarge) {
        maxOpt = 25;
        opt_it = 4;
    }
    const int Nd = std::min((int)pCurrentMap->KeyFramesInMap() - 2, maxOpt);
    const unsigned long maxKFid = pKF->mnId;

    vector<KeyFrame*> vpOptimizableKFs;
    const vector<KeyFrame*> vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
    list<KeyFrame*> lpOptVisKFs;

    vpOptimizableKFs.reserve(Nd);
    vpOptimizableKFs.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
    for (int i = 1; i < Nd; i++) {
        if (vpOptimizableKFs.back()->mPrevKF) {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
        } else
            break;
    }

    int N = vpOptimizableKFs.size();

    // Optimizable points seen by temporal optimizable keyframes
    list<MapPoint*> lLocalMapPoints;
    for (int i = 0; i < N; i++) {
        vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
        for (vector<MapPoint*>::iterator vit = vpMPs.begin(),
                                         vend = vpMPs.end();
             vit != vend; vit++) {
            MapPoint* pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnBALocalForKF != pKF->mnId) {
                        // 判断一下, 放置重新插入
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
        }
    }

    // Fixed Keyframe: First frame previous KF to optimization window)
    list<KeyFrame*> lFixedKeyFrames;
    if (vpOptimizableKFs.back()->mPrevKF) {
        lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
        vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
    } else {
        vpOptimizableKFs.back()->mnBALocalForKF = 0;
        vpOptimizableKFs.back()->mnBAFixedForKF = pKF->mnId;
        lFixedKeyFrames.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Optimizable visual KFs
    const int maxCovKF = 0;
    for (int i = 0, iend = vpNeighsKFs.size(); i < iend; i++) {
        if (lpOptVisKFs.size() >= maxCovKF) break;

        KeyFrame* pKFi = vpNeighsKFs[i];
        if (pKFi->mnBALocalForKF == pKF->mnId ||
            pKFi->mnBAFixedForKF == pKF->mnId)
            continue;
        pKFi->mnBALocalForKF = pKF->mnId;
        if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
            lpOptVisKFs.push_back(pKFi);

            vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
            for (vector<MapPoint*>::iterator vit = vpMPs.begin(),
                                             vend = vpMPs.end();
                 vit != vend; vit++) {
                MapPoint* pMP = *vit;
                if (pMP)
                    if (!pMP->isBad())
                        if (pMP->mnBALocalForKF != pKF->mnId) {
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF = pKF->mnId;
                        }
            }
        }
    }

    // Fixed KFs which are not covisible optimizable
    const int maxFixKF = 200;

    for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                   lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        map<KeyFrame*, tuple<int, int>> observations =
            (*lit)->GetObservations();
        for (map<KeyFrame*, tuple<int, int>>::iterator
                 mit = observations.begin(),
                 mend = observations.end();
             mit != mend; mit++) {
            KeyFrame* pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->mnId &&
                pKFi->mnBAFixedForKF != pKF->mnId) {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad()) {
                    lFixedKeyFrames.push_back(pKFi);
                    break;
                }
            }
        }
        if (lFixedKeyFrames.size() >= maxFixKF) break;
    }

    bool bNonFixed = (lFixedKeyFrames.size() == 0);

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType* linearSolver;
    linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

    if (bLarge) {
        g2o::OptimizationAlgorithmLevenberg* solver =
            new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(
            1e-2);  // to avoid iterating for finding optimal lambda
        optimizer.setAlgorithm(solver);
    } else {
        g2o::OptimizationAlgorithmLevenberg* solver =
            new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(1e0);
        optimizer.setAlgorithm(solver);
    }

    // Set Local temporal KeyFrame vertices
    N = vpOptimizableKFs.size();
    num_fixedKF = 0;
    num_OptKF = 0;
    num_MPs = 0;
    num_edges = 0;
    for (int i = 0; i < N; i++) {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        if (pKFi->bImu) {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
            VV->setFixed(false);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }
        num_OptKF++;
    }

    // Set Local visual KeyFrame vertices
    for (list<KeyFrame*>::iterator it = lpOptVisKFs.begin(),
                                   itEnd = lpOptVisKFs.end();
         it != itEnd; it++) {
        KeyFrame* pKFi = *it;
        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        num_OptKF++;
    }

    // Set Fixed KeyFrame vertices
    for (list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(),
                                   lend = lFixedKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame* pKFi = *lit;
        VertexPose* VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        if (pKFi->bImu)  // This should be done only for keyframe just before
                         // temporal window
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
            VV->setFixed(true);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
            VG->setFixed(true);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
            VA->setFixed(true);
            optimizer.addVertex(VA);
        }
        num_fixedKF++;
    }

    // Create intertial constraints
    vector<EdgeInertial*> vei(N, (EdgeInertial*)NULL);
    vector<EdgeGyroRW*> vegr(N, (EdgeGyroRW*)NULL);
    vector<EdgeAccRW*> vear(N, (EdgeAccRW*)NULL);

    for (int i = 0; i < N; i++) {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        if (!pKFi->mPrevKF) {
            cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << endl;
            continue;
        }
        if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 =
                optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
            g2o::HyperGraph::Vertex* VG1 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
            g2o::HyperGraph::Vertex* VA1 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
            g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
            g2o::HyperGraph::Vertex* VG2 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
            g2o::HyperGraph::Vertex* VA2 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);

            if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
                cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", "
                     << VA1 << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", "
                     << VA2 << endl;
                continue;
            }

            vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

            vei[i]->setVertex(
                0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            vei[i]->setVertex(
                1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            vei[i]->setVertex(
                2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
            vei[i]->setVertex(
                3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
            vei[i]->setVertex(
                4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            vei[i]->setVertex(
                5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

            if (i == N - 1 || bRecInit) {
                // All inertial residuals are included without robust cost
                // function, but not that one linking the
                // last optimizable keyframe inside of the local window and the
                // first fixed keyframe out. The
                // information matrix for this measurement is also downweighted.
                // This is done to avoid accumulating
                // error due to fixing variables.
                g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
                vei[i]->setRobustKernel(rki);
                if (i == N - 1)
                    vei[i]->setInformation(vei[i]->information() * 1e-2);
                rki->setDelta(sqrt(16.92));
            }
            optimizer.addEdge(vei[i]);

            vegr[i] = new EdgeGyroRW();
            vegr[i]->setVertex(0, VG1);
            vegr[i]->setVertex(1, VG2);
            cv::Mat cvInfoG =
                pKFi->mpImuPreintegrated->C.rowRange(9, 12).colRange(9, 12).inv(
                    cv::DECOMP_SVD);
            Eigen::Matrix3d InfoG;

            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    InfoG(r, c) = cvInfoG.at<float>(r, c);
            vegr[i]->setInformation(InfoG);
            optimizer.addEdge(vegr[i]);
            num_edges++;

            vear[i] = new EdgeAccRW();
            vear[i]->setVertex(0, VA1);
            vear[i]->setVertex(1, VA2);
            cv::Mat cvInfoA = pKFi->mpImuPreintegrated->C.rowRange(12, 15)
                                  .colRange(12, 15)
                                  .inv(cv::DECOMP_SVD);
            Eigen::Matrix3d InfoA;
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    InfoA(r, c) = cvInfoA.at<float>(r, c);
            vear[i]->setInformation(InfoA);

            optimizer.addEdge(vear[i]);
            num_edges++;
        } else
            cout << "ERROR building inertial edge" << endl;
    }

    // Set MapPoint vertices
    const int nExpectedSize =
        (N + lFixedKeyFrames.size()) * lLocalMapPoints.size();

    // Mono
    vector<EdgeMono*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    // Stereo
    vector<EdgeStereo*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float chi2Mono2 = 5.991;
    const float thHuberStereo = sqrt(7.815);
    const float chi2Stereo2 = 7.815;

    const unsigned long iniMPid = maxKFid * 5;

    map<int, int> mVisEdges;
    for (int i = 0; i < N; i++) {
        KeyFrame* pKFi = vpOptimizableKFs[i];
        mVisEdges[pKFi->mnId] = 0;
    }
    for (list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(),
                                   lend = lFixedKeyFrames.end();
         lit != lend; lit++) {
        mVisEdges[(*lit)->mnId] = 0;
    }

    num_MPs = lLocalMapPoints.size();
    for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                   lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));

        unsigned long id = pMP->mnId + iniMPid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        const map<KeyFrame*, tuple<int, int>> observations =
            pMP->GetObservations();

        // Create visual constraints
        for (map<KeyFrame*, tuple<int, int>>::const_iterator
                 mit = observations.begin(),
                 mend = observations.end();
             mit != mend; mit++) {
            KeyFrame* pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->mnId &&
                pKFi->mnBAFixedForKF != pKF->mnId)
                continue;

            if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
                const int leftIndex = get<0>(mit->second);

                cv::KeyPoint kpUn;

                // Monocular left observation
                if (leftIndex != -1 && pKFi->mvuRight[leftIndex] < 0) {
                    mVisEdges[pKFi->mnId]++;

                    kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono* e = new EdgeMono(0);
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    // Add here uncerteinty
                    const float unc2 = pKFi->mpCamera->uncertainty2(obs);
                    const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);
                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                    num_edges++;
                }
                // Stereo-observation
                else if (leftIndex != -1)  // Stereo observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    mVisEdges[pKFi->mnId]++;

                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    Eigen::Matrix<double, 3, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo* e = new EdgeStereo(0);
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                     optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    // Add here uncerteinty
                    const float unc2 = pKFi->mpCamera->uncertainty2(obs.head(2));
                    const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);

                    num_edges++;
                }

                // Monocular right observation
                if (pKFi->mpCamera2) {
                    int rightIndex = get<1>(mit->second);

                    if (rightIndex != -1) {
                        rightIndex -= pKFi->NLeft;
                        mVisEdges[pKFi->mnId]++;

                        Eigen::Matrix<double, 2, 1> obs;
                        cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        EdgeMono* e = new EdgeMono(1);

                        e->setVertex(
                            0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(id)));
                        e->setVertex(
                            1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                   optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                        const float& invSigma2 =
                            pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);

                        num_edges++;
                    }
                }
            }
        }
    }

    // cout << "Total map points: " << lLocalMapPoints.size() << endl;
    for (map<int, int>::iterator mit = mVisEdges.begin(),
                                 mend = mVisEdges.end();
         mit != mend; mit++) {
        assert(mit->second >= 3);
    }

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();

    float err = optimizer.activeRobustChi2();
    optimizer.optimize(opt_it);  // Originally to 2
    float err_end = optimizer.activeRobustChi2();
    if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

    vector<pair<KeyFrame*, MapPoint*>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // Check inlier observations
    // Mono
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        EdgeMono* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];
        bool bClose = pMP->mTrackDepth < 10.f;

        if (pMP->isBad()) continue;

        if ((e->chi2() > chi2Mono2 && !bClose) ||
            (e->chi2() > 1.5f * chi2Mono2 && bClose) || !e->isDepthPositive()) {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    // Stereo
    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        EdgeStereo* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad()) continue;

        if (e->chi2() > chi2Stereo2) {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    // Get Map Mutex and erase outliers
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if ((2 * err < err_end || isnan(err) || isnan(err_end)) && !bLarge) {
        cout << "FAIL LOCAL-INERTIAL BA!!!!" << endl;
        return;
    }

    if (!vToErase.empty()) {
        for (size_t i = 0; i < vToErase.size(); i++) {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Display main statistcis of optimization
    Verbose::PrintMess("LIBA KFs: " + to_string(N), Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("LIBA bNonFixed?: " + to_string(bNonFixed),
                       Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess(
        "LIBA KFs visual outliers: " + to_string(vToErase.size()),
        Verbose::VERBOSITY_DEBUG);

    for (list<KeyFrame*>::iterator lit = lFixedKeyFrames.begin(),
                                   lend = lFixedKeyFrames.end();
         lit != lend; lit++)
        (*lit)->mnBAFixedForKF = 0;

    // Recover optimized data
    // Local temporal Keyframes
    N = vpOptimizableKFs.size();
    for (int i = 0; i < N; i++) {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        cv::Mat Tcw =
            Converter::toCvSE3(VP->estimate().Rcw[0], VP->estimate().tcw[0]);
        pKFi->SetPose(Tcw);
        pKFi->mnBALocalForKF = 0;

        if (pKFi->bImu) {
            VertexVelocity* VV = static_cast<VertexVelocity*>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
            pKFi->SetVelocity(Converter::toCvMat(VV->estimate()));
            VertexGyroBias* VG = static_cast<VertexGyroBias*>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
            VertexAccBias* VA = static_cast<VertexAccBias*>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
            Vector6d b;
            b << VG->estimate(), VA->estimate();
            pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
        }
    }

    // Local visual KeyFrame
    for (list<KeyFrame*>::iterator it = lpOptVisKFs.begin(),
                                   itEnd = lpOptVisKFs.end();
         it != itEnd; it++) {
        KeyFrame* pKFi = *it;
        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        cv::Mat Tcw =
            Converter::toCvSE3(VP->estimate().Rcw[0], VP->estimate().tcw[0]);
        pKFi->SetPose(Tcw);
        pKFi->mnBALocalForKF = 0;
    }

    // Points
    for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(),
                                   lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
            optimizer.vertex(pMP->mnId + iniMPid + 1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    pMap->IncreaseChangeIndex();
}