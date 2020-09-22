//
// Created by panda on 20. 9. 19..
//

#ifndef SRC_ONLINE_INTERPOLATOR_H
#define SRC_ONLINE_INTERPOLATOR_H

#define ANSI_COLOR_RESET        "\x1b[0m"
#define ANSI_BOLD               "\x1b[1m"
#define ANSI_COLOR_RED          "\x1b[31m"
#define ANSI_COLOR_GREEN        "\x1b[32m"
#define ANSI_COLOR_YELLOW       "\x1b[33m"
#define ANSI_COLOR_BLUE         "\x1b[34m"
#define ANSI_COLOR_MAGENTA      "\x1b[35m"
#define ANSI_COLOR_CYAN         "\x1b[36m"

#define LOG_QOUT false // generates error
#define USE_GETQ_FUN false
#include <mutex>          // std::mutex

///////////////////////////////////////////////////////////////////////////////
/////////////////////// external communication thread /////////////////////////
///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <cstdio>
#include "stdio.h"
#include "stdlib.h"
#include <string>
#include "time.h"
#include "sys/types.h"
#include "sys/socket.h"
#include "netinet/in.h"
#include <pthread.h>
#include <arpa/inet.h>
#include "json/json.h"
#include <queue>
#include <Eigen/Eigen>

#define BUF_LEN 1024
#define WBUF_LEN 1024

#define PORT_REPEATER 1189

#define V_INTP_SATURATE 1.0
#define A_INTP_SATURATE 2.0

template<int DIM>
class OnlineInterpolator {
  public:
    typedef Eigen::Matrix<double, DIM, 1> JointVec;
    typedef Eigen::Matrix<double, DIM, 2> AlphaVec;
    struct JointVecState{
        JointVec y;
        JointVec dy;
        JointVec ddy;
    };

    Eigen::Matrix<double, 2, 2> TimeMatInv;
    std::deque<JointVec> Xqueue;
    std::deque<JointVec> Vqueue;
    std::deque<AlphaVec> Alphaqueue;
    std::deque<JointVec> Xout_queue;
    std::deque<JointVec> Vout_queue;
    std::deque<JointVec> Aout_queue;
    std::deque<double> time_queue;
    std::deque<int> step_queue;

    JointVec X0;
    JointVec V0;
    AlphaVec Alpha;
    double time0;

    double period_s = 1E-2;
    double period_update = 1E-2;
    double period_c;

    double k_gain=10.0;
    double d_gain=5.0;
    double alpha_lpf = 0.6;
    int step = 0;

    pthread_mutex_t mtx;           // mutex for critical section
    JointVecState lpf_Y;

  protected:
    pthread_t p_thread_online_interpolator = NULL;
    int thr_id_online_interpolator = NULL;

    Eigen::Matrix<double, 2, 2> TimeMat;
    JointVec lpf_x;
    JointVec lpf_y;

  public:
    void (*get_qcur_fun)(void*, double *);
    void* p_handle;

    void get_qcur(double *qcur){
        if (!USE_GETQ_FUN)
            printf(ANSI_COLOR_RED   "[Trajectory Server] USE_GETQ_FUN is disabled.\n"  ANSI_COLOR_RESET);
        get_qcur_fun(p_handle, qcur);
    }

    void set_qreader_fun(void (*_get_qcur)(void*, double *), void* _p_handle){
        if (!USE_GETQ_FUN)
            printf(ANSI_COLOR_RED   "[Trajectory Server] USE_GETQ_FUN is disabled.\n"  ANSI_COLOR_RESET);
        get_qcur_fun = _get_qcur;
        p_handle = _p_handle;
    }

    int reset(double _period_c, double _period_s, double* _qcur_in=NULL) {
        double _qcur[DIM];
        if (_qcur_in==NULL){
            get_qcur(_qcur);
        }
        else{
            for (int i_dim = 0; i_dim < DIM; i_dim++) {
                _qcur[i_dim] = _qcur_in[i_dim];
            }
        }
        pthread_mutex_lock(&mtx);
        for (int i_dim = 0; i_dim < DIM; i_dim++) {
            time0 = 0;
            X0(i_dim,0) = _qcur[i_dim];
            V0(i_dim,0) = 0;
            Alpha(i_dim, 0) = 0;
            Alpha(i_dim, 1) = 0;
            lpf_x(i_dim, 0) = 0;
            lpf_y(i_dim, 0) = 0;
            lpf_Y.y(i_dim, 0) = 0;
            lpf_Y.dy(i_dim, 0) = 0;
            lpf_Y.ddy(i_dim, 0) = 0;
        }
        Xqueue.clear();
        Vqueue.clear();
        Alphaqueue.clear();
        period_c = _period_c;
        if (LOG_QOUT){
            Xout_queue.clear();
            Vout_queue.clear();
            Aout_queue.clear();
            time_queue.clear();
            step_queue.clear();
        }
        pthread_mutex_unlock(&mtx);
        return set_sampling_period(_period_s);
    }

    int set_sampling_period(double _period_s){
        pthread_mutex_lock(&mtx);
        period_s = _period_s;
        period_update = period_s;
        TimeMat << period_s * period_s * period_s, period_s * period_s, \
                    8 * period_s * period_s * period_s, 4 * period_s * period_s;
        TimeMatInv << TimeMat.inverse();
        pthread_mutex_unlock(&mtx);
        return round(period_s/period_c);
    }

    bool stop() {
        pthread_mutex_lock(&mtx);
        JointVec Vtmp;
        AlphaVec Alphatmp;
        for (int i_dim = 0; i_dim < DIM; i_dim++) {
            Vtmp(i_dim,0) = 0;
            Alphatmp(i_dim, 0) = 0;
            Alphatmp(i_dim, 1) = 0;
        }
        if (Xqueue.empty()){
            Xqueue.push_back(X0);
        }
        else{
            Xqueue.push_back(Xqueue.back());
        }
        Vqueue.push_back(Vtmp);
        Alphaqueue.push_back(Alphatmp);
        pthread_mutex_unlock(&mtx);
        return true;
    }

    int push_next_qs(double *qs_next) {
        JointVec Xnew;
        for(int i_dim=0;i_dim<DIM;i_dim++) {
            Xnew(i_dim, 0) = qs_next[i_dim];
        }
//        std::cout<< "Xnew" << std::endl << Xnew <<std::endl;

        Xqueue.push_back(Xnew); // --> Xqueue[2]
        int qcount = Xqueue.size();
        if (qcount < 3){
            return qcount;
        }

        if(Vqueue.empty()){
            pthread_mutex_lock(&mtx);
            JointVec Xtmp;
            JointVec Vtmp;
            JointVec Atmp;
            calc_xva(period_update, X0, V0, Alpha, Xtmp, Vtmp, Atmp);
            Xqueue[0] << Xtmp;
            // Xtmp == Xqueue[0]

            Eigen::Matrix<double, 2, 1> Alphatmp;
            AlphaVec Alphavectmp;
            for(int i_dim=0;i_dim<DIM;i_dim++){
                Alphatmp = calc_alpha(Xtmp(i_dim, 0), Vtmp(i_dim, 0), Xqueue[1](i_dim, 0), Xqueue[2](i_dim, 0));
                Alphavectmp(i_dim,0) = Alphatmp(0,0);
                Alphavectmp(i_dim,1) = Alphatmp(1,0);
            }
//        std::cout<< "Xtmp" << std::endl << Xtmp <<std::endl;
//        std::cout<< "Vtmp" << std::endl << Vtmp <<std::endl;
//        std::cout<< "Alphavectmp" << std::endl << Alphavectmp <<std::endl;
            Vqueue.push_back(Vtmp); // --> Vqueue[0]
            Alphaqueue.push_back(Alphavectmp); // --> Alphaqueue[0]
            pthread_mutex_unlock(&mtx);
        }
        return qcount;
    }

    Eigen::Matrix<double, 2, 1> calc_alpha(double x0, double v0, double x1, double x2) {
        Eigen::Matrix<double, 2, 1> Alpha;
        Eigen::Matrix<double, 2, 1> Pvec;
        Pvec << x1 - x0 - v0 * period_s, x2 - x0 - 2 * v0 * period_s;
        Alpha << TimeMatInv * Pvec;
        return Alpha;
    }

    double calc_xva(double t, JointVec & X0, JointVec & V0, AlphaVec & Alpha, JointVec &pd, JointVec &vd, JointVec &ad) {
        double v0, x0, alpha, beta;
        for (int i_dim = 0; i_dim < DIM; i_dim++) {
            x0 = X0(i_dim,0);
            v0 = V0(i_dim,0);
            alpha = Alpha(i_dim, 0);
            beta = Alpha(i_dim, 1);
            pd(i_dim, 0) = alpha * t * t * t + beta * t * t + v0 * t + x0;
            vd(i_dim, 0) = fmax(-V_INTP_SATURATE,fmin(V_INTP_SATURATE, 3 * alpha * t * t + 2 * beta * t + v0));
            ad(i_dim, 0) = fmax(-A_INTP_SATURATE,fmin(A_INTP_SATURATE,6 * alpha * t + 2 * beta));
        }
    }

    void get_next_qc(double time, JointVec &pd, JointVec &vd, JointVec &ad) {
        pthread_mutex_lock(&mtx);
        if(time - time0 > period_update){
            if (!Xqueue.empty()) {
                JointVec Xtmp = Xqueue.front();
                Xqueue.pop_front();
                if (!(Vqueue.empty() || Alphaqueue.empty())) {
                    step += 1;
                    period_update = 0;
                    time0 = period_s*floor(time/period_s);
                    X0 = Xtmp;
                    V0 = Vqueue.front();
                    Alpha = Alphaqueue.front();
                    Vqueue.pop_front();
                    Alphaqueue.pop_front();
                }
                else{
                    calc_xva(period_update, X0, V0, Alpha, pd, vd, ad);
                    period_update = 0;
                    time0 = period_s*floor(time/period_s);
                    X0 = Xtmp;
                    for (int i_dim = 0; i_dim < DIM; i_dim++) {
                        V0(i_dim, 0) = vd(i_dim, 0)/2;
                        Alpha(i_dim, 0) = 0;
                        Alpha(i_dim, 1) = 0;
                    }
                }
            }
            else{
                calc_xva(period_update, X0, V0, Alpha, pd, vd, ad);
                period_update = 0;
                time0 = period_s*floor(time/period_s);
                for (int i_dim = 0; i_dim < DIM; i_dim++) {
                    V0(i_dim, 0) = vd(i_dim, 0)/2;
                    Alpha(i_dim, 0) = 0;
                    Alpha(i_dim, 1) = 0;
                }
            }
            period_update = period_s*ceil((time - time0)/period_s);
        }
        calc_xva(time - time0, X0, V0, Alpha, pd, vd, ad);
        if (LOG_QOUT){
            Xout_queue.push_back(pd);
            Vout_queue.push_back(vd);
            Aout_queue.push_back(ad);
            step_queue.push_back(step);
            time_queue.push_back(time);
        }
        pthread_mutex_unlock(&mtx);
    }

    void set_lpfY(double* Y) {
        for (int i_dim = 0; i_dim < DIM; i_dim++) {
            lpf_Y.y(i_dim, 0) = Y[i_dim];
            lpf_Y.dy(i_dim, 0) = 0;
            lpf_Y.ddy(i_dim, 0) = 0;
        }
    }

    JointVecState& lpf_joint_state(JointVec& Y) {
        JointVecState Ypre;
        Ypre.y << lpf_Y.y;
        Ypre.dy << lpf_Y.dy;
        Ypre.ddy << lpf_Y.ddy;
        lpf_Y.y << (1 - alpha_lpf) * Ypre.y + alpha_lpf * Y;
        lpf_Y.dy << (lpf_Y.y - Ypre.y)/period_c;
        double vmax = lpf_Y.dy.abs().maxCoeff();
        if (vmax > V_INTP_SATURATE){
            lpf_Y.dy << (lpf_Y.dy/vmax*V_INTP_SATURATE);
            lpf_Y.y << (Ypre.y + lpf_Y.dy*period_c);
        }
        lpf_Y.ddy << (lpf_Y.dy - Ypre.dy)/period_c;
        double amax = lpf_Y.ddy.abs().maxCoeff();
        if (amax > A_INTP_SATURATE){
            lpf_Y.ddy << (lpf_Y.ddy/amax*A_INTP_SATURATE);
            lpf_Y.dy << (Ypre.dy + lpf_Y.ddy*period_c);
            lpf_Y.y << (Ypre.y + lpf_Y.dy*period_c);
        }
        return lpf_Y;
    }

    JointVec& lpf(JointVec& X){
        JointVec y;
        y << (k_gain*X + d_gain*(X - lpf_x));
        lpf_x << X;
        lpf_y << (1 - alpha_lpf) * lpf_y + alpha_lpf * y;
        return lpf_y;
    }

    void init_thread(double _period_c, double _period_s, double* qval=NULL);
};

template<int DIM>
void *socket_thread_vel(void *arg) {
    OnlineInterpolator<DIM> *jpr;
    jpr = (OnlineInterpolator<DIM> *) arg;
    char buffer[BUF_LEN];
    char wbuffer[WBUF_LEN];
    struct sockaddr_in server_addr, client_addr;
    char temp[20];
    int server_fd, client_fd;
    //server_fd, client_fd : 각 소켓 번호
    socklen_t len, msg_size;

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {// 소켓 생성
        printf(ANSI_COLOR_RED   "[Trajectory Server] Can't open stream socket\n"    ANSI_COLOR_RESET);
        exit(0);
    }
    memset(&server_addr, 0x00, sizeof(server_addr));
    //server_Addr 을 NULL로 초기화

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(PORT_REPEATER);
    //server_addr 셋팅

    int reusePort = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &reusePort, sizeof(reusePort));

    if (bind(server_fd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {//bind() 호출
        printf(ANSI_COLOR_RED   "[Trajectory Server] Can't bind local address.\n"   ANSI_COLOR_RESET);
        return nullptr;
    }

    if (listen(server_fd, 5) < 0) {//소켓을 수동 대기모드로 설정
        printf(ANSI_COLOR_RED   "[Trajectory Server] Can't listening connect.\n"    ANSI_COLOR_RESET);
        exit(0);
    }

    memset(buffer, 0x00, sizeof(buffer));
    printf(ANSI_COLOR_CYAN   "[Trajectory Server] wating connection request.\n"  ANSI_COLOR_RESET);
    len = sizeof(client_addr);
    bool terminate = false;
    while (!terminate) {
        Json::Value send_json;
        Json::Value read_json;
        client_fd = accept(server_fd, (struct sockaddr *) &client_addr, &len);
        if (client_fd < 0) {
            printf(ANSI_COLOR_RED   "[Trajectory Server] accept failed.\n"  ANSI_COLOR_RESET);
            exit(0);
        }
        inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, temp, sizeof(temp));
//        printf("[Trajectory Server] %s client connected.\n", temp);

        msg_size = read(client_fd, buffer, BUF_LEN);
//        printf("[Trajectory Server] read %d bytes.\n", (int) msg_size);

        Json::CharReaderBuilder builder;
        builder.settings_["indentation"] = "";
        Json::CharReader *reader(builder.newCharReader());
        std::string errs;
        bool parsingRet = reader->parse((char *) buffer, (char *) buffer + strlen(buffer), &read_json, &errs);

        Json::Value::Members members = read_json.getMemberNames();

        double val;
        double qval[DIM];
        double period_s;
        int step_c;
        typename OnlineInterpolator<DIM>::JointVec pd, vd, ad;
        Json::Value qval_s;
        int qcount;
        bool ret;

        for (const auto &membername : members) {
            if (strcmp(membername.c_str(), "reset") == 0) {
                if (USE_GETQ_FUN){
                    step_c = jpr->reset(jpr->period_c, read_json["period_s"].asFloat());
                }
                else{
                    for (int i = 0; i < DIM; i++) {
                        qval[i] = read_json["qcur"][i].asFloat();
                    }
                    step_c = jpr->reset(jpr->period_c, read_json["period_s"].asFloat(), qval);
                }
                send_json["step_c"] = step_c;
            } else if (strcmp(membername.c_str(), "stop") == 0) {
                ret = jpr->stop();
                send_json["stop"] = ret;
            } else if (strcmp(membername.c_str(), "terminate") == 0) {
                terminate = true;
                send_json["terminate"] = true;
            } else if(strcmp(membername.c_str(), "getq") == 0){
                jpr->get_qcur(qval);
                for (int i = 0; i < DIM; i++) {
                    qval_s.append(qval[i]);
                }
                send_json["qval"] = qval_s;
            } else if (strcmp(membername.c_str(), "qval") == 0) {
                for (int i = 0; i < DIM; i++) {
                    qval[i] = read_json["qval"][i].asFloat();
                    qval_s.append(qval[i]);
                }
                qcount = jpr->push_next_qs(qval);
                send_json["qval"] = qval_s;
                send_json["qcount"] = qcount;
            }else if (strcmp(membername.c_str(), "qcount") == 0) {
                send_json["qcount"] = jpr->Xqueue.size();
            }else if (strcmp(membername.c_str(), "k_gain") == 0) {
                val = read_json["k_gain"].asFloat();
                if (val>=0)
                    jpr->k_gain = val;
                send_json["k_gain"] = jpr->k_gain;
            }else if (strcmp(membername.c_str(), "d_gain") == 0) {
                val = read_json["d_gain"].asFloat();
                if (val>=0)
                    jpr->d_gain = val;
                send_json["d_gain"] = jpr->d_gain;
            }else if (strcmp(membername.c_str(), "alpha_lpf") == 0) {
                val = read_json["alpha_lpf"].asFloat();
                if (val>=0)
                    jpr->alpha_lpf = val;
                send_json["alpha_lpf"] = jpr->alpha_lpf;
            }
        }

        Json::StreamWriterBuilder wbuilder;
        std::stringstream read_stream;
        wbuilder.settings_["indentation"] = "";
        Json::StreamWriter *writer(wbuilder.newStreamWriter());
        std::string str;

        str = writer->write(send_json, &read_stream);
        str = read_stream.str();
//        printf("[Trajectory Server] return %d bytes.\n", str.length());
        memcpy(wbuffer, str.c_str(), str.length());
        write(client_fd, wbuffer, str.length());
        close(client_fd);
//        printf("[Trajectory Server] %s client closed.\n", temp);
    }
    close(server_fd);
    printf(ANSI_COLOR_CYAN   "[Trajectory Server] socket connection closed.\n"  ANSI_COLOR_RESET);


    pthread_mutex_destroy(&jpr->mtx);
    if (LOG_QOUT){
        std::string filePath = "../test.txt";

        // write File
        std::ofstream writeFile(filePath.data());
        if( writeFile.is_open() ){
            printf(ANSI_COLOR_RED   "[Trajectory Server] write log file.\n"  ANSI_COLOR_RESET);
            for(int i_q=0; i_q<jpr->Xout_queue.size(); i_q++){
                writeFile << jpr->step_queue[i_q] << "\t" ;
                writeFile << jpr->time_queue[i_q] << "\t" ;
                for(int i_dim=0;i_dim<DIM; i_dim++){
                    writeFile << jpr->Xout_queue[i_q][i_dim] << "\t" ;
                }
                for(int i_dim=0;i_dim<DIM; i_dim++){
                    writeFile << jpr->Vout_queue[i_q][i_dim] << "\t" ;
                }
                for(int i_dim=0;i_dim<DIM; i_dim++){
                    writeFile << jpr->Aout_queue[i_q][i_dim] << "\t" ;
                }
                writeFile << std::endl;
            }
            writeFile.close();
            printf(ANSI_COLOR_RED   "[Trajectory Server] writing log done.\n"  ANSI_COLOR_RESET);
        }
    }

    return nullptr;
}
///////////////////////////////////////////////////////////////////////////////
/////////////////////// external communication thread /////////////////////////
///////////////////////////////////////////////////////////////////////////////

template<int DIM>
void OnlineInterpolator<DIM>::init_thread(double _period_c, double _period_s, double* qval){
    reset(_period_c, _period_s, qval);

    if (thr_id_online_interpolator == NULL){
        pthread_mutex_init(&mtx, NULL);
        thr_id_online_interpolator = pthread_create(&p_thread_online_interpolator, NULL, socket_thread_vel<DIM>, (void *) this);
        if (thr_id_online_interpolator < 0) {
            printf(ANSI_COLOR_RED "[Trajectory Server] thread create error" ANSI_COLOR_RESET);
            exit(0);
        }
    }
}
#endif //SRC_ONLINE_INTERPOLATOR_H
