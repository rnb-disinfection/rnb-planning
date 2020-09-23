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
#include <queue>
#include <Eigen/Eigen>
#include "json/json.h"
#include "gason.h"

#define BUF_LEN 1024
#define WBUF_LEN 1024

#define PORT_REPEATER 1189

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

    double V_SATURATE = 1.0;
    double A_SATURATE = 2.0;

    double period_s = 2E-2;
    double period_c;

    double k_gain=50.0;
    double d_gain=5.0;
    double alpha_lpf = 0.8;
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
    void (*get_qcur_fun)(void*, double *) = nullptr;
    void* p_handle;

    void get_qcur(double *qcur){
        if (get_qcur_fun == nullptr) {
            printf(ANSI_COLOR_RED   "[Trajectory Server] get_qcur_fun is not defined.\n"  ANSI_COLOR_RESET);
            for (int i_dim = 0; i_dim < DIM; i_dim) {
                qcur[i_dim] = 0.0;
            }
        }
        else{
            get_qcur_fun(p_handle, qcur);
        }
    }

    void set_qreader_fun(void (*_get_qcur)(void*, double *), void* _p_handle){
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
        set_lpf_joint_state(_qcur);
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

       pthread_mutex_lock(&mtx);
       Xqueue.push_back(Xnew); // --> Xqueue[2]
       int qcount = Xqueue.size();
       if (qcount < 3){
           pthread_mutex_unlock(&mtx);
           return qcount;
       }

       if(Vqueue.empty()){
           JointVec Xtmp;
           JointVec Vtmp;
           JointVec Atmp;
           calc_xva(period_s, X0, V0, Alpha, Xtmp, Vtmp, Atmp);
           Xqueue[0] << Xtmp;
           // Xtmp == Xqueue[0]

           Eigen::Matrix<double, 2, 1> Alphatmp;
           AlphaVec Alphavectmp;
           for(int i_dim=0;i_dim<DIM;i_dim++){
               Alphatmp = calc_alpha(Xqueue[0](i_dim, 0), Vtmp(i_dim, 0), Xqueue[1](i_dim, 0), Xqueue[2](i_dim, 0));
               Alphavectmp(i_dim,0) = Alphatmp(0,0);
               Alphavectmp(i_dim,1) = Alphatmp(1,0);
           }
//        std::cout<< "Xtmp" << std::endl << Xtmp <<std::endl;
//        std::cout<< "Vtmp" << std::endl << Vtmp <<std::endl;
//        std::cout<< "Alphavectmp" << std::endl << Alphavectmp <<std::endl;
           Vqueue.push_back(Vtmp); // --> Vqueue[0]
           Alphaqueue.push_back(Alphavectmp); // --> Alphaqueue[0]
       }
       pthread_mutex_unlock(&mtx);
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
            vd(i_dim, 0) = fmax(-V_SATURATE,fmin(V_SATURATE, 3 * alpha * t * t + 2 * beta * t + v0));
            ad(i_dim, 0) = fmax(-A_SATURATE,fmin(A_SATURATE,6 * alpha * t + 2 * beta));
        }
    }

    void get_next_qc(double time, JointVec &pd, JointVec &vd, JointVec &ad, bool apply_state_lpf=false) {
        for (int i_dim = 0; i_dim < DIM; i_dim++) {
            pd(i_dim, 0) = 0;
            vd(i_dim, 0) = 0;
            ad(i_dim, 0) = 0;
        }
       pthread_mutex_lock(&mtx);
       if(time - time0 > period_s){
           if (!Xqueue.empty()) {
               JointVec Xtmp = Xqueue.front();
               Xqueue.pop_front();
               if (!(Vqueue.empty() || Alphaqueue.empty())) {
                   step += 1;
                   X0 = Xtmp;
                   V0 = Vqueue.front();
                   Alpha = Alphaqueue.front();
                   Vqueue.pop_front();
                   Alphaqueue.pop_front();
               }
               else{
                   calc_xva(period_s, X0, V0, Alpha, pd, vd, ad);
                   X0 = Xtmp;
                   for (int i_dim = 0; i_dim < DIM; i_dim++) {
                       V0(i_dim, 0) = vd(i_dim, 0)/2;
                       Alpha(i_dim, 0) = 0;
                       Alpha(i_dim, 1) = 0;
                   }
               }
           }
           else{
               calc_xva(period_s, X0, V0, Alpha, pd, vd, ad);
               for (int i_dim = 0; i_dim < DIM; i_dim++) {
                   X0(i_dim, 0) = pd(i_dim, 0);
                   V0(i_dim, 0) = vd(i_dim, 0)/2;
                   Alpha(i_dim, 0) = 0;
                   Alpha(i_dim, 1) = 0;
               }
           }
           time0 = period_s*floor(time/period_s);
       }
       calc_xva(time - time0, X0, V0, Alpha, pd, vd, ad);
       if (apply_state_lpf)
           lpf_joint_state(pd, vd, ad);
       if (LOG_QOUT){
           Xout_queue.push_back(pd);
           Vout_queue.push_back(vd);
           Aout_queue.push_back(ad);
           step_queue.push_back(step);
           time_queue.push_back(time);
       }
       pthread_mutex_unlock(&mtx);
    }

    void set_lpf_joint_state(double* Y) {
        for (int i_dim = 0; i_dim < DIM; i_dim++) {
            lpf_Y.y(i_dim, 0) = Y[i_dim];
            lpf_Y.dy(i_dim, 0) = 0;
            lpf_Y.ddy(i_dim, 0) = 0;
        }
    }

    JointVecState& lpf_joint_state(JointVec& Y, JointVec& dY, JointVec& ddY) {
        JointVecState Ypre;
        Ypre.y << lpf_Y.y;
        Ypre.dy << lpf_Y.dy;
        Ypre.ddy << lpf_Y.ddy;
        lpf_Y.y << (1 - alpha_lpf) * Ypre.y + alpha_lpf * Y;
        lpf_Y.dy << (lpf_Y.y - Ypre.y)/period_c;
        double vmax = lpf_Y.dy.cwiseAbs().maxCoeff();
        if (vmax > V_SATURATE){
            lpf_Y.dy << (lpf_Y.dy/vmax*V_SATURATE);
            lpf_Y.y << (Ypre.y + lpf_Y.dy*period_c);
        }
        lpf_Y.ddy << (lpf_Y.dy - Ypre.dy)/period_c;
        double amax = lpf_Y.ddy.cwiseAbs().maxCoeff();
        if (amax > A_SATURATE){
            lpf_Y.ddy << (lpf_Y.ddy/amax*A_SATURATE);
            lpf_Y.dy << (Ypre.dy + lpf_Y.ddy*period_c);
            lpf_Y.y << (Ypre.y + lpf_Y.dy*period_c);
        }
        Y<<lpf_Y.y;
        dY<<lpf_Y.dy;
        ddY<<lpf_Y.ddy;
        return lpf_Y;
    }

    JointVec& kd_lpf(JointVec& X){
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
    char wbuffer[WBUF_LEN];
    char buffer[BUF_LEN];
    struct sockaddr_in server_addr, client_addr;
    char temp[32];
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
    printf(ANSI_COLOR_CYAN   "[Trajectory Server] wating connection request.\n"  ANSI_COLOR_RESET);
    len = sizeof(client_addr);
    bool terminate = false;
    while (!terminate) {
        JsonValue read_json;
        JsonAllocator allocator;
        char* endptr;
        Json::Value send_json;
        std::string errs;

        client_fd = accept(server_fd, (struct sockaddr *) &client_addr, &len);
        if (client_fd < 0) {
            printf(ANSI_COLOR_RED   "[Trajectory Server] accept failed.\n"  ANSI_COLOR_RESET);
            exit(0);
        }
        inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, temp, sizeof(temp));
//        printf(ANSI_COLOR_BLUE"[Trajectory Server] %s client connected.\n" ANSI_COLOR_RESET, temp);

        memset(buffer, 0x00, sizeof(buffer));
        msg_size = read(client_fd, buffer, BUF_LEN);
//        printf(ANSI_COLOR_BLUE "[Trajectory Server] read %d bytes.\n" ANSI_COLOR_RESET, (int) msg_size);
//        printf(buffer);
//        printf("\n");

       int parsingRet = jsonParse(buffer, &endptr, &read_json, allocator);

       if (parsingRet != JSON_OK){
           printf(ANSI_COLOR_RED   "[Trajectory Server]  parse error %s at $zd.\n"  ANSI_COLOR_RESET,
                  jsonStrError(parsingRet), endptr - buffer);
           continue;
       }

        double val;
        int step_c;
        typename OnlineInterpolator<DIM>::JointVec pd, vd, ad;
        Json::Value qval_s;
        int qcount;
        bool ret;

        bool _reset = false;

        double _qval[DIM];
        double _period_s;

        for (auto i : read_json) {
            if (strcmp(i->key, "reset") == 0) {
                _reset = true;
            } else if (strcmp(i->key, "period_s") == 0) {
                _period_s = i->value.toNumber();
            } else if (strcmp(i->key, "qcur") == 0) {
                int i_dim = 0;
                for (auto i_q:i->value) {
                    _qval[i_dim] = i_q->value.toNumber();
                    i_dim ++;
                }
            } else if (strcmp(i->key, "stop") == 0) {
                ret = jpr->stop();
                send_json["stop"] = ret;
            } else if (strcmp(i->key, "terminate") == 0) {
                terminate = true;
                send_json["terminate"] = true;
            } else if (strcmp(i->key, "getq") == 0) {
                jpr->get_qcur(_qval);
                for (int i_dim = 0; i_dim < DIM; i_dim++) {
                    qval_s.append(_qval[i_dim]);
                }
                send_json["qval"] = qval_s;
            } else if (strcmp(i->key, "qval") == 0) {
                int i_dim = 0;
                for (auto i_q:i->value) {
                    _qval[i_dim] = i_q->value.toNumber();
                    qval_s.append(_qval[i_dim]);
                    i_dim ++;
                }
                qcount = jpr->push_next_qs(_qval);
                send_json["qval"] = qval_s;
                send_json["qcount"] = qcount;
            } else if (strcmp(i->key, "qcount") == 0) {
               send_json["qcount"] = jpr->Xqueue.size();
            } else if (strcmp(i->key, "k_gain") == 0) {
                val = i->value.toNumber();
                if (val>=0)
                    jpr->k_gain = val;
                send_json["k_gain"] = jpr->k_gain;
            } else if (strcmp(i->key, "d_gain") == 0) {
                val = i->value.toNumber();
                if (val>=0)
                    jpr->d_gain = val;
                send_json["d_gain"] = jpr->d_gain;
            } else if (strcmp(i->key, "alpha_lpf") == 0) {
                val = i->value.toNumber();
                if (val>=0)
                    jpr->alpha_lpf = val;
                send_json["alpha_lpf"] = jpr->alpha_lpf;
            } else if (strcmp(i->key, "v_sat") == 0) {
                val = i->value.toNumber();
                if (val>=0)
                    jpr->V_SATURATE = val;
                send_json["v_sat"] = jpr->V_SATURATE;
            } else if (strcmp(i->key, "a_sat") == 0) {
                val = i->value.toNumber();
                if (val>=0)
                    jpr->A_SATURATE = val;
                send_json["a_sat"] = jpr->A_SATURATE;
            }
        }
        if (_reset) {
            if (jpr->get_qcur_fun == nullptr){
                step_c = jpr->reset(jpr->period_c, _period_s, _qval);
            }
            else{
                step_c = jpr->reset(jpr->period_c, _period_s);
            }
            send_json["step_c"] = step_c;
        }

        Json::StreamWriterBuilder wbuilder;
        std::stringstream read_stream;
        wbuilder.settings_["indentation"] = "";
        Json::StreamWriter *writer(wbuilder.newStreamWriter());
        std::string str;

        str = writer->write(send_json, &read_stream);
        str = read_stream.str();
//        printf("[Trajectory Server] return %d bytes.\n", str.length());
        memset(wbuffer, 0x00, sizeof(wbuffer));
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
        //pthread_mutex_init(&mtx, NULL);
        thr_id_online_interpolator = pthread_create(&p_thread_online_interpolator, NULL, socket_thread_vel<DIM>, (void *) this);
        if (thr_id_online_interpolator < 0) {
            printf(ANSI_COLOR_RED "[Trajectory Server] thread create error" ANSI_COLOR_RESET);
            exit(0);
        }
    }
}
#endif //SRC_ONLINE_INTERPOLATOR_H
