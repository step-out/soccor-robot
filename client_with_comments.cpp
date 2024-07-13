#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "compress.h"

#include <rcssbase/net/socketstreambuf.hpp>
#include <rcssbase/net/udpsocket.hpp>
#include <rcssbase/gzip/gzstream.hpp>

#ifdef HAVE_SSTREAM
#include <sstream>
#else
#include <strstream>
#endif
#include <iostream>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <math.h>
#include <algorithm>

#ifdef __CYGWIN__
// cygwin is not win32
#elif defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
#define RCSS_WIN
#include <winsock2.h>
#endif

#ifndef RCSS_WIN
#include <unistd.h>     // select()
#include <sys/select.h> // select()
#include <sys/time.h>   // select()
#include <sys/types.h>  // select()
#endif

using namespace std;

enum PlayerRole
{
    goalkeeper,
    defender,
    centre_forword,
    striker
};

const double PI = 3.14159265359;           // 圆周率
const char TEAM_NAME[20] = "soccor-eater"; // 球队名称
const double MARGIN = 0.7;                 // 可踢范围0.7米
const double RATE = 0.027;
const double PLAYER_SIZE = 0.3; // 球员大小
const double BALL_SIZE = 0.085; // 球的大小
const double MAX_SPEED = 1;     // 球员最大速度
const double DASH_POWER_RATE = 0.006;
/*-----比赛的全局信息----*/
char play_mode[30];    // 比赛模式
int current_cycle = 0; // 当前周期
int last_cycle = 0;    // 上一个周期
/*-----球员自身信息----*/
int id = 1;                // 球员ID
PlayerRole role = striker; 
bool side = 0;             // 左或右
double inix = -50;    // 初始化在场上的位置
double iniy = 0;
double sidex = -50; // 考虑左或右的全局位置
double sidey = 0;
double x = -50; // 球员的位置
double y = 0;
bool infield = 1;    // 是否在场内
double minx = -52.5; // 限定球员范围
double maxx = 52.5;
double miny = -34;
double maxy = 34;
double speed = 0;              // 速度
double speed_to_head_dir = 0;  // 速度相对头部的角度
double speed_global_angle = 0; // 速度的绝对方向
double head_to_body_angle = 0; // 头部相对身体的角度
double head_global_angle = 0;  // 头部的绝对方向
double body_global_angle = 0;  // 身体的绝对方向
double view_angle = 90;        // 视野范围
char view_width[10];           // 视野宽度
double stamina = 8000;         // 体力
double effort = 1;             // 体力使用效率
int see_mate_num = 0;          // 见到的队友人数
int see_oppo_num = 0;          // 见到的对手人数
bool ball_on_me = 0;           // 球是否在自己这里
bool ball_on_side = 0;         // 球是否在自己这一队
bool ball_on_opposide = 0;     // 球是否在对手那里
bool catch_moved = 0;          // 守门员扑球成功之后是否移动
int catch_wait = 0;            // 扑球成功之后等待一会
int kick_times = 0;            // 踢球的次数
int catch_times = 0;           // 扑球的次数
bool kicked = 0;               // 上一个周期是否踢到球
bool catched = 0;              // 上一个周期是否扑到球
double kickdir = 0;            // 踢球方向
double goaldir, goaldist;      // 球门方向和距离
double goal_x, goal_y;
double goal_tx, goal_ty;
double goal_bx, goal_by;
double get_ball_x = 0; // 截球点
double get_ball_y = 0;
int wait = 0;     // 等待时间，此期间不移动
bool waited = 0;  // 是否等待了
char command[50]; // 向server发送的消息
/*-----球的信息----*/
struct Ball // 球
{
    bool visible;              // 能否看见
    bool moving;               // 是否在移动
    double dist_change;        // 距离的相对变化
    double dir_change;         // 方向的相对变化
    double x[50];              // x坐标
    double y[50];              // y坐标
    double dist;               // 距离
    double dir;                // 方向
    double speed;              // 球速
    double speed_global_angle; // 球运动的绝对角度
} ball;
/*-----球员信息----*/
struct Player // 球员
{
    double x;
    double y;
    double dist;
    double dir;
    double dist_change;
    double dir_change;
} teammates[11], opponents[22]; // 队友和对手，不能区分的算作对手
/*-----标志信息----*/
struct Flags // 标志
{
    bool visible; // 能否看见
    double dist;  // 距离
    double dir;   // 方向
} flags[55];      // 55个标志
char flagName[55][20] =
    {
        "(g l)", "(f g l t)", "(f g l b)", "(g r)", "(f g r t)",
        "(f g r b)", "(f c)", "(f c t)", "(f c b)", "(f p l c)",
        "(f p l t)", "(f p l b)", "(f p r c)", "(f p r t)", "(f p r b)",
        "(f l t)", "(f l b)", "(f r t)", "(f r b)", "(f l t 30)",
        "(f l t 20)", "(f l t 10)", "(f l 0)", "(f l b 10)", "(f l b 20)",
        "(f l b 30)", "(f b l 50)", "(f b l 40)", "(f b l 30)", "(f b l 20)",
        "(f b l 10)", "(f b 0)", "(f b r 10)", "(f b r 20)", "(f b r 30)",
        "(f b r 40)", "(f b r 50)", "(f r b 30)", "(f r b 20)", "(f r b 10)",
        "(f r 0)", "(f r t 10)", "(f r t 20)", "(f r t 30)", "(f t r 50)",
        "(f t r 40)", "(f t r 30)", "(f t r 20)", "(f t r 10)", "(f t 0)",
        "(f t l 10)", "(f t l 20)", "(f t l 30)", "(f t l 40)", "(f t l 50)"}; // 标志名称
struct Flag_coord
{
    double x;
    double y;
}; // 标志的绝对坐标
struct Flag_coord flag_coord[55] =
    {
        {-52.50, 0.0000},
        {-52.5, -7.32},
        {-52.5, 7.320},
        {52.500, 0.0000},
        {52.50, -7.32},
        {52.500, 7.3200},
        {0.000, 0.000},
        {0.000, -34.0},
        {0.0000, 34.000},
        {-36.0, 0.000},
        {-36.00, -20.16},
        {-36.0, 20.16},
        {36.00, 0.000},
        {36.000, -20.16},
        {36.00, 20.16},
        {-52.50, -34.00},
        {-52.5, 34.00},
        {52.50, -34.0},
        {52.500, 34.000},
        {-57.5, -30.0},
        {-57.50, -20.00},
        {-57.5, -10.0},
        {-57.5, 0.000},
        {-57.50, 10.000},
        {-57.5, 20.00},
        {-57.50, 30.000},
        {-50.0, 39.00},
        {-40.0, 39.00},
        {-30.00, 39.000},
        {-20.0, 39.00},
        {-10.00, 39.000},
        {0.000, 39.00},
        {10.00, 39.00},
        {20.000, 39.000},
        {30.00, 39.00},
        {40.000, 39.000},
        {50.00, 39.00},
        {57.50, -30.0},
        {57.500, -20.00},
        {57.50, -10.0},
        {57.500, 0.0000},
        {57.50, 10.00},
        {57.50, 20.00},
        {57.500, 30.000},
        {50.00, -39.0},
        {40.000, -39.00},
        {30.00, -39.0},
        {20.00, -39.0},
        {10.000, -39.00},
        {0.000, -39.0},
        {-10.00, -39.00},
        {-20.0, -39.0},
        {-30.0, -39.0},
        {-40.00, -39.00},
        {-50.0, -39.0}};
/*-----边线信息----*/
struct Lines // 边线
{
    bool visible;
    double dist;
    double dir;
} lines[4];
char line_name[4][6] = {"(l l)", "(l r)", "(l t)", "(l b)"}; // 边线名称
double line_global_angle[4] = {-180, 0, -90, 90};            // 边线的全局角度
/*-----全局辅助函数----*/
double AngToRad(double x) // 角度转弧度
{
    return x * PI / 180.0;
}
double RadToAng(double x) // 弧度转角度
{
    return x * 180.0 / PI;
}
double AddAng(double x, double y) // 角度相加
{
    if (x + y > -180 && x + y <= 180)
        return x + y;
    else if (x + y <= -180)
        return x + y + 360;
    else
        return x + y - 360;
}
double SubAng(double x, double y) // 角度相减
{
    if (x - y > -180 && x - y <= 180)
        return x - y;
    else if (x - y <= -180)
        return x - y + 360;
    else
        return x - y - 360;
}
double CalDist(double x1, double y1, double x2, double y2) //(x1,y2)到(x2,y2)的距离
{
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
double CalDir(double x1, double y1, double x2, double y2) //(x1,y2)到(x2,y2)的全局方向
{
    if (x1 == x2 && y1 == y2)
        return 0;
    if (x1 == x2)
    {
        if (y1 < y2)
            return 90;
        else
            return -90;
    }
    else if (y1 == y2)
    {
        if (x1 < x2)
            return 0;
        else
            return 180;
    }
    else
    {
        if (x2 > x1)
            return RadToAng(atan((y2 - y1) / (x2 - x1)));
        else
        {
            if (y2 < y1)
                return RadToAng(atan((y2 - y1) / (x2 - x1))) - 180.0;
            else
                return RadToAng(atan((y2 - y1) / (x2 - x1))) + 180.0;
        }
    }
}
double RandBetween(double a, double b) // 产生[a,b]之间的随机浮点数
{
    srand(time(0));
    return 1.0 * rand() / RAND_MAX * (b - a) + a;
}
class Client
{
private:
    rcss::net::Addr M_dest;
    rcss::net::UDPSocket M_socket;
    rcss::net::SocketStreamBuf *M_socket_buf;
    rcss::gz::gzstreambuf *M_gz_buf;
    std::ostream *M_transport;
    int M_comp_level;
    bool M_clean_cycle;
#ifdef HAVE_LIBZ
    Decompressor M_decomp;
#endif
public:
    Client(const std::string &server, const int port) : M_dest(port), M_socket(), M_socket_buf(NULL), M_gz_buf(NULL),
                                                        M_transport(NULL), M_comp_level(-1), M_clean_cycle(true)
    {
        M_dest.setHost(server);
        Open();
        Bind();
        M_socket_buf->setEndPoint(M_dest);
    }
    virtual ~Client()
    {
        Close();
    }
    int Open()
    {
        if (M_socket.open())
        {
            if (M_socket.setNonBlocking() < 0)
            {
                std::cerr << __FILE__ << ": " << __LINE__ << ": Error setting socket non-blocking: " << strerror(errno) << std::endl;
                M_socket.close();
                return -1;
            }
        }
        else
        {
            std::cerr << __FILE__ << ": " << __LINE__ << ": Error opening socket: " << strerror(errno) << std::endl;
            M_socket.close();
            return -1;
        }

        M_socket_buf = new rcss::net::SocketStreamBuf(M_socket);
        M_transport = new std::ostream(M_socket_buf);
        return 0;
    }
    bool Bind()
    {
        if (!M_socket.bind(rcss::net::Addr()))
        {
            std::cerr << __FILE__ << ": " << __LINE__ << ": Error connecting socket" << std::endl;
            M_socket.close();
            return false;
        }
        return true;
    }
    void Close()
    {
        M_socket.close();
        if (M_transport)
        {
            delete M_transport;
            M_transport = NULL;
        }
        if (M_gz_buf)
        {
            delete M_gz_buf;
            M_gz_buf = NULL;
        }
        if (M_socket_buf)
        {
            delete M_socket_buf;
            M_socket_buf = NULL;
        }
    }
    int SetCompression(int level)
    {
#ifdef HAVE_LIBZ
        if (level >= 0)
        {
            if (!M_gz_buf)
            {
                M_gz_buf = new rcss::gz::gzstreambuf(*M_socket_buf);
            }
            M_gz_buf->setLevel(level);
            M_transport->rdbuf(M_gz_buf);
        }
        else
        {
            M_transport->rdbuf(M_socket_buf);
        }
        return M_comp_level = level;
#endif
        return M_comp_level = -1;
    }
    void ProcessMsg(char *msg, int len)
    {
#ifdef HAVE_LIBZ
        if (M_comp_level >= 0)
        {
            M_decomp.decompress(msg, len, Z_SYNC_FLUSH);
            char *out;
            int size;
            M_decomp.getOutput(out, size);
            if (size > 0)
            {
                ParseMsg(out);
            }
        }
        else
#endif
        {
            ParseMsg(msg);
        }
    }
    void MessageLoop()
    {
        fd_set read_fds;
        fd_set read_fds_back;
        char buf[8192];
        memset(&buf, 0, sizeof(char) * 8192);
        int in = fileno(stdin);
        FD_ZERO(&read_fds);
        FD_SET(in, &read_fds);
        FD_SET(M_socket.getFD(), &read_fds);
        read_fds_back = read_fds;
#ifdef RCSS_WIN
        int max_fd = 0;
#else
        int max_fd = M_socket.getFD() + 1;
#endif
        while (1)
        {
            read_fds = read_fds_back;
            int ret = ::select(max_fd, &read_fds, NULL, NULL, NULL);
            if (ret < 0)
            {
                perror("Error selecting input");
                break;
            }
            else if (ret != 0)
            {
                if (FD_ISSET(in, &read_fds))
                {
                    if (std::fgets(buf, sizeof(buf), stdin) != NULL)
                    {
                        size_t len = std::strlen(buf);
                        if (buf[len - 1] == '\n')
                        {
                            buf[len - 1] = '\0';
                            --len;
                        }

                        M_transport->write(buf, len + 1);
                        M_transport->flush();
                        if (!M_transport->good())
                        {
                            if (errno != ECONNREFUSED)
                            {
                                std::cerr << __FILE__ << ": " << __LINE__ << ": Error sending to socket: " << strerror(errno) << std::endl
                                          << "msg = [" << buf << "]\n";
                            }
                            M_socket.close();
                        }
                        std::cout << buf << std::endl;
                    }
                }
                if (FD_ISSET(M_socket.getFD(), &read_fds))
                {
                    rcss::net::Addr from;
                    int len = M_socket.recv(buf, sizeof(buf) - 1, from);
                    if (len == -1 && errno != EWOULDBLOCK)
                    {
                        if (errno != ECONNREFUSED)
                        {
                            std::cerr << __FILE__ << ": " << __LINE__ << ": Error receiving from socket: " << strerror(errno) << std::endl;
                        }
                        M_socket.close();
                    }
                    else if (len > 0)
                    {
                        M_dest.setPort(from.getPort());
                        M_socket_buf->setEndPoint(M_dest);
                        ProcessMsg(buf, len);
                    }
                }
            }
        }
    }
    // 向server发送命令
    bool SendCmd(char *command);
    // 解析server发送的信息
    void Init(char *msg);      // 初始化
    void Hear(char *msg);      // Hear，获知比赛模式
    void SenseBody(char *msg); // SenseBody
    void See(char *msg);       // See，看到标志、边线、球、球员等信息
    // 球员基础行为命令
    void Kick(double power, double direction);
    void Dash(double power, double direction);
    void Turn(double moment);
    void Turn_neck(double moment);
    void Tackle(double power);
    void Move(double x, double y);
    void Catch(double direction);
    void ChangeView(const char *width, const char *quality);
    // 辅助函数
    void UpdateINFO();                // 更新信息
    bool turn(double angle);          // 转过一定角度
    bool GotoPos(double x, double y); // 定点前往(x,y)
    bool BallINField();               // 球是否在自己的活动范围内
    bool BallINPenalty();             // 球是否在己方禁区内
    bool BallINOppopenalty();         // 球是否在对方禁区内
    bool CanGoal();                   // 能否射门，并给出射门角度
    int BallWay();                    // 分析球的走向，不动或者远离返回0，否则返回1
    void ParseMsg(char *msg);
    void Run();
};
namespace
{
    Client *client = static_cast<Client *>(0);
    void SigExitHandle(int)
    {
        std::cerr << "\nKilled. Exiting..." << std::endl;
        if (client)
        {
            delete client;
            client = static_cast<Client *>(0);
        }
        std::exit(EXIT_FAILURE);
    }
}
bool Client::SendCmd(char *command)
{
    int len;
    len = strlen(command) + 1;
    printf("command:%s\n", command);
    M_transport->write(command, len);
    M_transport->flush();
    if (!M_transport->good())
    {
        printf("error send socket\n");
        return false;
    }
    return true;
}
void Client::Init(char *msg) // 初始化
{
    /*
    Init函数用于根据传入的消息字符串进行初始化操作。
    根据消息中的角色信息（左方或右方），设置相关变量的初始值，
    包括位置坐标、比赛模式等。根据角色的不同，还进行了一些特定的坐标和目标位置的设置。
    */
    char Side;
    // (init Side UniformNumber PlayMode)
    sscanf(msg, "(init %c %d %s)", &Side, &id, play_mode);
    play_mode[strlen(play_mode) - 1] = 0;
    if (Side == 'l') // 字符'l'表示自己是左方
    {
        side = 0;
        x = sidex = inix;
        y = sidey = iniy;
        goal_x = flag_coord[3].x; // 对于左方而言goal是右边的球门
        goal_y = flag_coord[3].y;
        goal_tx = flag_coord[4].x;
        goal_ty = flag_coord[4].y;
        goal_bx = flag_coord[5].x;
        goal_by = flag_coord[5].y;
    }
    else // 右方
    {
        side = 1;
        x = sidex = -inix;
        y = sidey = -iniy;
        swap(minx, maxx);
        swap(miny, maxy);
        minx = -minx;
        maxx = -maxx;
        miny = -miny;
        maxy = -maxy;
        goal_x = flag_coord[0].x; // 对于右方而言goal是左边的球门
        goal_y = flag_coord[0].y;
        goal_tx = flag_coord[1].x;
        goal_ty = flag_coord[1].y;
        goal_bx = flag_coord[2].x;
        goal_by = flag_coord[2].y;
    }
}
void Client::Hear(char *msg) // Hear，获知比赛模式
{
    sscanf(msg, "(hear %d referee %s)", &current_cycle, play_mode);
    play_mode[strlen(play_mode) - 1] = 0; // 去掉右括号')'
}
void Client::SenseBody(char *msg) // SenseBody
{
    int last_kick = kick_times;
    int last_catch = catch_times;
    sscanf(msg, "(sense_body %d (view_mode %*s %s (stamina %lf %lf) (speed %lf %lf) (head_angle %lf) "
                "(kick %d) (dash %*d) (Turn %*d) (say %*d) (turn_neck %*d) (catch %d)",
           &current_cycle, view_width, &stamina, &effort,
           &speed, &speed_to_head_dir, &head_to_body_angle, &kick_times, &catch_times);
    view_width[strlen(view_width) - 1] = 0;

    if (!strcmp(view_width, "wide"))
        view_angle = 180; // 宽视野视角为180度
    else if (!strcmp(view_width, "normal"))
        view_angle = 90; // 普通视野视角为90度
    else
        view_angle = 45; // 窄视野视角为45度

    if (kick_times == last_kick + 1)
        kicked = 1; // 踢球次数加1，说明上一个周期踢到了球
    else
        kicked = 0;

    if (catch_times == last_catch + 1)
        catched = 1; // 扑球次数加1，说明上一个周期扑到了球
}
void Client::See(char *msg) // see，看到标志、边线、球、球员等信息
{
    sscanf(msg, "(see %d", &current_cycle);
    char *p = 0;
    char format[40];
    char str[30];
    for (int i = 0; i < 55; i++)
    {
        p = strstr(msg, flagName[i]); // 找标志
        if (p != 0)
        {
            flags[i].visible = 1; // 可见
            sprintf(format, "%s %%lf %%lf", flagName[i]);
            sscanf(p, format, &flags[i].dist, &flags[i].dir); // 获取标志距离和方向
        }
        else
        {
            flags[i].visible = 0; // 否则不可见
        }
    }
    for (int i = 0; i < 4; i++)
    {
        p = strstr(msg, line_name[i]); // 找边线
        if (p != 0)
        {
            lines[i].visible = 1;
            sprintf(format, "%s %%lf %%lf", line_name[i]);
            sscanf(p, format, &lines[i].dist, &lines[i].dir);
        }
        else
        {
            lines[i].visible = 0;
        }
    }
    sprintf(str, "(p"); // 找球员
    p = strstr(msg, str);
    int mate_i = 0, oppo_i = 0;
    for (int i = 0; p; i++)
    {
        p += 2;
        if (*p == ')') // 无法确定球员属于哪一边，视为对手
        {
            p++;
            sscanf(p, "%lf %lf", &opponents[oppo_i].dist, &opponents[oppo_i].dir);
            oppo_i++;
        }
        else
        {
            p += 1; // 跳过一个空格
            char tmp[50];
            sscanf(p, "%s", tmp);
            if(tmp[0] == '"')
            {
                tmp[strlen(tmp) - 1] = '\0';
                if(tmp[strlen(tmp) - 1] == '"')
                    tmp[strlen(tmp) - 1] = '\0';
            }
            p = strstr(p, ")");
            if (!strcmp(tmp, TEAM_NAME)) // 队友
            {
                teammates[mate_i].dist_change = teammates[mate_i].dir_change = 0;
                sscanf(p, "%*s %lf %lf %lf %lf", &teammates[mate_i].dist, &teammates[mate_i].dir, &teammates[mate_i].dist_change, &teammates[mate_i].dir_change);
                mate_i ++;
            }
            else // 对手
            {
                opponents[oppo_i].dist_change = opponents[oppo_i].dir_change = 0;
                sscanf(p, "%*s %lf %lf %lf %lf", &opponents[oppo_i].dist, &opponents[oppo_i].dir, &opponents[oppo_i].dist_change, &opponents[oppo_i].dir_change);
                oppo_i ++;
            }
        }
        p = strstr(p, str);
    }
    see_mate_num = mate_i;
    see_oppo_num = oppo_i;

    sprintf(str, "(b)");
    p = strstr(msg, str); // 找球
    if (p == 0)
    {
        sprintf(str, "(B)"); // 邻域以内的球
        p = strstr(msg, str);
    }
    if (p != 0)
    {
        ball.visible = 1;
        ball.dist_change = 0;
        ball.dir_change = 0;
        sprintf(format, "%s %%lf %%lf %%lf %%lf", str);
        sscanf(p, format, &ball.dist, &ball.dir, &ball.dist_change, &ball.dir_change);
    }
    else
    {
        ball.visible = 0;
    }
    sprintf(str, "(G)"); // 找邻域的球门
    p = strstr(msg, str);
    if (p != 0)
    {
        sprintf(format, "%s %%lf %%lf", str);
        if (x < 0) // 左侧球门
        {
            flags[0].visible = 1;
            sscanf(p, format, &flags[0].dist, &flags[0].dir);
        }
        else // 右侧球门
        {
            flags[3].visible = 1;
            sscanf(p, format, &flags[3].dist, &flags[3].dir);
        }
    }
}
void Client::Kick(double power, double direction)
{
    /*
    power: 踢球的力量大小决定对球的加速大小
    direction: 踢球的角度
    球员的最大控球范围半径 = BALL_SIZE(0.085) + PLAYER_SIZE(0.3) + KICKABLE_MARGIN(0.7) = 1.085
    实际踢球力量act_pow = power * (1 - 0.25 * (dir_diff / 180) - 0.25 * (dist_diff / kickable_MARGIN)
    dir_diff是球和球员身体方向之间的绝对角度
    dist_diff是球和球员之间的距离(球员和球圆边界之间的距离)
    球获得的加速度 = min(act_pow * KICKPOWERRATE(0.027), ball_accel_max = MAXPOWER * KICKPOWERRATE)
    BALL_ACCEL_MAX = 2.7
    球最大速度BALL_SPEED_MAX = 3.0
    球的速度衰减BALL_DECAY = 0.94
    球速噪声 BALL_RAND(0.05) * (v + a)
    */
    sprintf(command, "(kick %lf %lf)", power, direction);
}
void Client::Dash(double power, double direction)
{
    /*
    power [-100, 100]
    power > 0: 球员向前加速
    power < 0: 球员向后加速
    执行dash命令会消耗体力: power > 0, 消耗power; power < 0, 消耗-2 * power
    实际加速力量cat_pow = power * effort
    实际加速度 = act_pow * DASHPOWERRATE(0.006)
    可以得出最大加速度 = 0.006 * 100 = 0.6
    球员速度噪声 PLAYER_RAND(0.1) * (v + a)
    球员最大速度PLAYER_SPEED_MAX = 1.05
    球员速度衰减PLAYER_DECAY = 0.4
    注意：球员不能在同一个周期同时执行dash和Turn两个命令
    */
    /*-----体力模型-----
    包含三个部分: stamina(体力值), effort(加速效率), recovery(体力恢复的速率)
    stamina  [0, 8000]
    每周期恢复体力 = recovery * STAMINA_INC_MAX(45)
    effort   [0.6, 1.0]
    如果体力值低于恢复速率递减阈值, 则恢复速率会减小, 每周期减小RECOVERY_DEC = 0.002
    恢复速率递减阈值 = RECOVERY_DEC_THR(0.3) * STAMINA_MAX(8000)
    recovery [0.5, 1]
    如果体力值低于加速效率递减阈值, 则加速效率会降低, 每周期降低EFFORT_DEC = 0.005
    加速效率递减阈值 = EFFORT_DEC_THR(0.3) * STAMINA_MAX(8000)
    如果体力值高于加速效率递增阈值, 则加速效率会提高, 每周期增加EFFORT_INC = 0.01
    加速效率递增阈值 = EFFORT_INC_THR(0.6) * STAMINA_MAX(8000)
    */
    sprintf(command, "(dash %lf %lf)", power, direction);
}
void Client::Turn(double moment)
{
    /*
    实际转身角度act_ang = ((1.0 + r) * moment) / (1.0 + inertia_moment * player_speed)
    r [-PLAYER_RAND, PLAYER_RAND], PLAYER_RAND = 0.1
    球员惯性大小参数inertia_moment = IMPARAM = 5.0
    注意：球员不能在同一个周期同时执行dash和Turn两个命令
    */
    sprintf(command, "(turn %lf)", moment);
}
void Client::Turn_neck(double moment)
{
    sprintf(command, "(turn_neck %lf)", moment);
}
void Client::Tackle(double power)
{
    /*
    power: 铲球的力量
    */
    sprintf(command, "(tackle %lf)", power);
}
void Client::Move(double x, double y)
{
    sprintf(command, "(move %lf %lf)", x, y);
}
void Client::Catch(double direction)
{
    /*
    direction: 扑球的角度，[-180, 180]
    */
    sprintf(command, "(catch %lf)", direction);
}
void Client::ChangeView(const char *width, const char *quality)
{
    /*
    width: 视野范围，wide(2)、normal(1)、narrow(0.5)
    quality：视觉质量，high(1)、low(0.5)
    实际视野范围: view_angle = visible_angle(90) * view_width_factor
    视觉刷新频率: view_frequency = sense_step(150 ms) * view_quality_factor * view_width_factor
    */
    sprintf(command, "(change_view %s %s)", width, quality);
}
void Client::UpdateINFO() // 更新信息
{
    int seelinenum = 0, j = 0;
    for (int i = 0; i < 4; i++)
    {
        if (lines[i].visible)
        {
            seelinenum++;
            if (seelinenum != 1)
            {
                j = lines[j].dist > lines[i].dist ? j : i; // 选择距离远的边线
                break;
            }
            else
                j = i;
        }
    }
    if (seelinenum != 1)
        infield = 0;                                                                                                  // 看到超过两条边线，说明自己在场外
    head_global_angle = SubAng(line_global_angle[j], (lines[j].dir < 0) ? (lines[j].dir + 90) : (lines[j].dir - 90)); // 确定头部的绝对方向
    body_global_angle = SubAng(head_global_angle, head_to_body_angle);                                                // 确定身体的绝对方向
    speed_global_angle = AddAng(speed_to_head_dir, head_global_angle);                                                // 确定速度的绝对方向
    double mindist = 1000;
    for (int i = 0; i < 55; i++)
    {
        if (flags[i].visible && flags[i].dist < mindist)
        {
            j = i;
            mindist = flags[i].dist;
        }
    } // 选择距离最近的标志来确定自己的位置
    x = flag_coord[j].x - flags[j].dist * cos(AngToRad(AddAng(flags[j].dir, head_global_angle)));
    y = flag_coord[j].y - flags[j].dist * sin(AngToRad(AddAng(flags[j].dir, head_global_angle)));
    for (int i = 0; i < see_mate_num; i++) // 确定队友的位置
    {
        teammates[i].x = x + teammates[i].dist * cos(AngToRad(AddAng(teammates[i].dir, head_global_angle)));
        teammates[i].y = y + teammates[i].dist * sin(AngToRad(AddAng(teammates[i].dir, head_global_angle)));
    }
    for (int i = 0; i < see_oppo_num; i++) // 确定对手的位置
    {
        opponents[i].x = x + opponents[i].dist * cos(AngToRad(AddAng(opponents[i].dir, head_global_angle)));
        opponents[i].y = y + opponents[i].dist * sin(AngToRad(AddAng(opponents[i].dir, head_global_angle)));
    }
    if (ball.visible) // 如果能看见球，确定球的位置
    {
        ball.x[0] = ball.x[1]; // 上一个周期球的位置
        ball.y[0] = ball.y[1];
        ball.x[1] = x + ball.dist * cos(AngToRad(AddAng(ball.dir, head_global_angle))); // 当前周期球的位置
        ball.y[1] = y + ball.dist * sin(AngToRad(AddAng(ball.dir, head_global_angle)));
        if (CalDist(ball.x[0], ball.y[0], ball.x[1], ball.y[1]) < 0.1)
            ball.moving = 0; // 速度小于0.1视为没有移动
        else
            ball.moving = 1;
        ball.speed_global_angle = CalDir(ball.x[0], ball.y[0], ball.x[1], ball.y[1]); // 确定球的速度方向
        for (int i = 2; i < 50; i++)
        { // 预测球在接下来48个周期内的位置
            ball.x[i] = ball.x[i - 1] + CalDist(ball.x[i - 2], ball.y[i - 2], ball.x[i - 1], ball.y[i - 1]) * 0.94 * cos(AngToRad(ball.speed_global_angle));
            ball.y[i] = ball.y[i - 1] + CalDist(ball.x[i - 2], ball.y[i - 2], ball.x[i - 1], ball.y[i - 1]) * 0.94 * sin(AngToRad(ball.speed_global_angle));
        }
        ball.speed = CalDist(ball.x[1], ball.y[1], ball.x[2], ball.y[2]); // 确定球的速度
        if (ball.dist <= MARGIN + BALL_SIZE + PLAYER_SIZE)                // 球在控球范围之内
        {
            ball_on_me = 1;
        }
        else
        {
            ball_on_me = 0;
            ball_on_side = 0;
            ball_on_opposide = 0;
            for (int i = 0; i < see_oppo_num; i++)
            {
                if (CalDist(opponents[i].x, opponents[i].y, ball.x[1], ball.y[1]) <= MARGIN + BALL_SIZE + PLAYER_SIZE)
                {
                    ball_on_opposide = 1; // 球在对手那里
                }
            }
            if (!ball_on_opposide)
            {
                for (int i = 0; i < see_mate_num; i++) // 球在队友那里
                {
                    if (CalDist(teammates[i].x, teammates[i].y, ball.x[1], ball.y[1]) <= MARGIN + BALL_SIZE + PLAYER_SIZE)
                    {
                        ball_on_side = 1;
                    }
                }
            }
        }
    }
}
bool Client::turn(double angle) // 转过一定角度
{
    static int turn_times = 0;
    static double turn_angle = 0;
    if (turn_times == 0)
    {
        turn_angle = angle; // 第一次转
    }
    if (fabs(turn_angle) < 3) // 小于3度就不转
    {
        turn_times = 0;
        turn_angle = 0;
        return 0;
    }
    turn_times++;
    double fact_turn = turn_angle / (1.0 + 5.0 * speed); // 实际转过的角度
    Turn(turn_angle);
    turn_angle = SubAng(turn_angle, fact_turn);
    return 1;
}
bool Client::GotoPos(double x, double y) // 定点前往(x,y)
{
    if (CalDist(x, y, x, y) < MARGIN)
        return 0;
    else
    {
        Dash(100, SubAng(CalDir(x, y, x, y), body_global_angle));
        return 1;
    }
}
bool Client::BallINField() // 球是否在自己的活动范围内
{
    if (ball.x[1] >= minx && ball.x[1] <= maxx && ball.y[1] >= miny && ball.y[1] <= maxy)
        return 1;
    else
        return 0;
}
bool Client::BallINPenalty() // 球是否在己方禁区内
{
    if (side == 0)
    {
        if (ball.x[1] >= -52.5 && ball.x[1] <= -36 && ball.y[1] >= -20.16 && ball.y[1] <= 20.16)
            return 1;
        else
            return 0;
    }
    else
    {
        if (ball.x[1] >= 36 && ball.x[1] <= 52.5 && ball.y[1] >= -20.16 && ball.y[1] <= 20.16)
            return 1;
        else
            return 0;
    }
}
bool Client::BallINOppopenalty() // 球是否在对方禁区内
{
    if (side == 0)
    {
        if (ball.x[1] >= 36 && ball.x[1] <= 52.5 && ball.y[1] >= -20.16 && ball.y[1] <= 20.16)
            return 1;
        else
            return 0;
    }
    else
    {
        if (ball.x[1] >= -52.5 && ball.x[1] <= -36 && ball.y[1] >= -20.16 && ball.y[1] <= 20.16)
            return 1;
        else
            return 0;
    }
}
bool Client::CanGoal() // 能否射门，并给出射门角度
{
    double ang1, ang2;
    goaldist = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));
    ang1 = SubAng(CalDir(x, y, goal_tx, goal_ty), body_global_angle);
    ang2 = SubAng(CalDir(x, y, goal_bx, goal_by), body_global_angle);
    if (ang1 > ang2)
        swap(ang1, ang2);
    if (ang2 - ang1 > 180)
        goaldir = ang1;
    else
        goaldir = RandBetween(ang1 + 1, ang2 - 1);
    if (goaldist > 30)
        return 0;
    else
        return 1;
}
int Client::BallWay() // 分析球的走向，不动或者远离返回0，否则返回1
{
    if (!ball.moving)
        return 0;
    if (CalDist(x, y, ball.x[1], ball.y[1]) <= CalDist(x, y, ball.x[2], ball.y[2]))
    {
        return 0;
    }
    double mindist = ball.dist;
    for (int i = 2; i < 50; i++) // 给出距离自己最近的点作为截球点
    {
        if (CalDist(x, y, ball.x[i], ball.y[i]) < mindist)
        {
            mindist = CalDist(x, y, ball.x[i], ball.y[i]);
            if (fabs(ball.x[i]) > 52.5 || fabs(ball.y[i]) > 34)
            {
                get_ball_x = ball.x[i - 1];
                get_ball_y = ball.y[i - 1];
                break;
            }
            else
            {
                get_ball_x = ball.x[i];
                get_ball_y = ball.y[i];
            }
        }
    }
    return 1;
}
void Client::ParseMsg(char *msg)
{
    if (!strncmp(msg, "(init", 5)) // init
    {
        Init(msg);
        ChangeView("narrow", "high");
        SendCmd(command);
        return;
    }
    else if (!strncmp(msg, "(hear", 5)) // Hear
    {
        Hear(msg);
    }
    else if (!strncmp(msg, "(sense_body", 11)) // SenseBody
    {
        SenseBody(msg);
    }
    else if (!strncmp(msg, "(see", 4)) // See
    {
        See(msg);
        UpdateINFO(); // 更新信息
    }
    else
        return;

    if (last_cycle != current_cycle)
    {
        last_cycle = current_cycle; // 新周期，直接返回，避免一个周期发送过多消息
        return;
    }

    if (!strcmp(play_mode, "before_kick_off") || !strncmp(play_mode, "goal_l", 6) || !strncmp(play_mode, "goal_r", 6))
    {
        // 开球之前或者进球之后，回到初始化地点
        if (CalDist(x, y, sidex, sidey) > 1)
        {
            Move(inix, iniy);
            SendCmd(command);
            return;
        }
        else if (turn(SubAng(CalDir(x, y, 0, 0), body_global_angle))) // 面向(0,0)的位置
        {
            SendCmd(command);
            return;
        }
        else
            return;
    }
    else if (!strcmp(play_mode, "kick_off_l") && side == 0) // 我方开球
    {
        if (id == 11) // 11号球员开球
        {
            if (GotoPos(-0.4, 0))
            {
                SendCmd(command);
                return;
            }
            if (RandBetween(0, 1) > 0.5)
                kickdir = SubAng(CalDir(x, y, 3, 20), body_global_angle);
            else
                kickdir = SubAng(CalDir(x, y, 3, -20), body_global_angle);
            Kick(80, kickdir);
            SendCmd(command);
            return;
        }
        else
            return;
    }
    else if (!strcmp(play_mode, "kick_off_r") && side == 1) // 我方开球
    {
        if (id == 11) // 11号球员开球
        {
            if (GotoPos(0.4, 0))
            {
                SendCmd(command);
                return;
            }
            if (RandBetween(0, 1) > 0.5)
                kickdir = SubAng(CalDir(x, y, -3, 20), body_global_angle);
            else
                kickdir = SubAng(CalDir(x, y, -3, -20), body_global_angle);
            Kick(80, kickdir);
            SendCmd(command);
            return;
        }
        else
            return;
    }
    else if ((side == 0 && (!strcmp(play_mode, "free_kick_l") || !strcmp(play_mode, "goal_kick_l") ||
                            !strcmp(play_mode, "indirect_free_kick_l") || !strcmp(play_mode, "kick_in_l") || !strcmp(play_mode, "corner_kick_l"))) ||
             (side == 1 && (!strcmp(play_mode, "free_kick_r") || !strcmp(play_mode, "goal_kick_r") ||
                            !strcmp(play_mode, "indirect_free_kick_r") || !strcmp(play_mode, "kick_in_r") || !strcmp(play_mode, "corner_kick_r"))))
    { // 我方发球
        if (BallINPenalty())
        {
            if (role == goalkeeper)
            {
                if (catched)
                {
                    if (wait == 0 && !waited)
                    {
                        wait = 45;
                        return;
                    }
                    else if (wait)
                    {
                        wait--;
                        if (wait == 0)
                            waited = 1;
                        else
                            return;
                    }
                    if (!catch_moved)
                    {
                        catch_moved = 1;
                        if (side)
                            Move(RandBetween(-maxx + 2, -minx - 2), RandBetween(-maxy + 2, -miny - 2));
                        else
                            Move(RandBetween(minx + 2, maxx - 2), RandBetween(miny + 2, maxy - 2));
                        SendCmd(command);
                        return;
                    }
                    else
                    {
                        catch_moved = 0;
                        catched = 0;
                        waited = 0;
                        wait = 0;
                        // sprintf(command, "(kick 100 %lf)", SubAng(side ? 180 : 0, body_global_angle));
                        Kick(100, SubAng(side ? 180 : 0, body_global_angle));
                        SendCmd(command);
                        return;
                    }
                }
                else if (!ball.visible)
                {
                    // sprintf(command, "(turn %lf)", view_angle);
                    Turn(view_angle);
                    SendCmd(command);
                    return;
                }
                else if (turn(ball.dir))
                {
                    SendCmd(command);
                    return;
                }
                else if (GotoPos(ball.x[1], ball.y[1]))
                {
                    SendCmd(command);
                    return;
                }
                else
                {
                    // sprintf(command, "(kick 100 %lf)", SubAng(side ? 180 : 0, body_global_angle));
                    Kick(100, SubAng(side ? 180 : 0, body_global_angle));
                    SendCmd(command);
                    return;
                }
            }
        }
        else
        {
            if (role == centre_forword && stamina > 3500)
            {
                if (!ball.visible)
                {
                    // sprintf(command, "(turn %lf)", view_angle);
                    Turn(view_angle);
                    SendCmd(command);
                    return;
                }
                else if (turn(ball.dir))
                {
                    SendCmd(command);
                    return;
                }
                else if (GotoPos(ball.x[1], ball.y[1]))
                {
                    SendCmd(command);
                    return;
                }
                else
                {
                    CanGoal();
                    kickdir = goaldir;
                    // sprintf(command, "(kick 100 %lf)", kickdir);
                    Kick(100, kickdir);
                    SendCmd(command);
                    return;
                }
            }
        }
        if (role != striker)
        {
            if (CalDist(x, y, sidex, sidey) >= MARGIN && turn(SubAng(CalDir(x, y, sidex, sidey), body_global_angle)))
            {
                SendCmd(command);
                return;
            }
            else if (GotoPos(sidex, sidey))
            {
                SendCmd(command);
                return;
            }
        }
        if (!ball.visible)
        {
            // sprintf(command, "(turn %lf)", view_angle);
            Turn(view_angle);
            SendCmd(command);
            return;
        }
        else
            return;
    }
    else if ((side == 0 && (!strcmp(play_mode, "kick_off_r") || !strcmp(play_mode, "free_kick_r") || !strcmp(play_mode, "goal_kick_r") ||
                            !strcmp(play_mode, "corner_kick_r") || !strcmp(play_mode, "kick_in_r") || !strcmp(play_mode, "indirect_free_kick_r") ||
                            !strcmp(play_mode, "offside_l"))) ||
             (side == 1 && (!strcmp(play_mode, "kick_off_l") || !strcmp(play_mode, "free_kick_l") || !strcmp(play_mode, "goal_kick_l") ||
                            !strcmp(play_mode, "corner_kick_l") || !strcmp(play_mode, "kick_in_l") || !strcmp(play_mode, "indirect_free_kick_l") ||
                            !strcmp(play_mode, "offside_r"))))
    { // 对方发球
        if ((!side && !strcmp(play_mode, "kick_off_r")) || (side && !strcmp(play_mode, "kick_off_l")))
            return;
        if (role != striker)
        {
            if (CalDist(x, y, sidex, sidey) >= MARGIN && turn(SubAng(CalDir(x, y, sidex, sidey), body_global_angle)))
            {
                SendCmd(command);
                return;
            }
            else if (GotoPos(sidex, sidey))
            {
                SendCmd(command);
                return;
            }
        }
        if (!ball.visible)
        {
            // sprintf(command, "(turn %lf)", view_angle);
            Turn(view_angle);
            SendCmd(command);
            return;
        }
        else
            return;
    }
    if (role == goalkeeper) // 守门员
    {
        if (ball.visible && ball.dist <= 2) // 扑球
        {
            if (fabs(ball.dir) > 90)
            {
                // sprintf(command, "(dash 100 %lf)", ball.dir);
                Dash(100, ball.dir);
                SendCmd(command);
                return;
            }
            else
            {
                // sprintf(command, "(catch %lf)", AddAng(head_to_body_angle, ball.dir));
                Catch(AddAng(head_to_body_angle, ball.dir));
                SendCmd(command);
                return;
            }
        }
        else if (!ball.visible)
        {
            // sprintf(command, "(turn %lf)", view_angle);
            Turn(view_angle);
            SendCmd(command);
            return;
        }
        else if (fabs(ball.dir + ball.dir_change) > view_angle / 2) // 球会超出视野
        {
            // sprintf(command, "(turn %lf)", ball.dir + ball.dir_change);
            Turn(ball.dir + ball.dir_change);
            SendCmd(command);
            return;
        }
        else if (BallINPenalty()) // 球在禁区内
        {
            if (ball.speed > 1) // 球速大于1，选择扑球
            {
                if (BallWay())
                {
                    if (GotoPos(get_ball_x, get_ball_y))
                    {
                        SendCmd(command);
                        return;
                    }
                    else
                        return;
                }
                else
                {
                    // sprintf(command, "(turn %lf)", ball.dir);
                    Turn(ball.dir);
                    SendCmd(command);
                    return;
                }
            }
            else // 球速小于1，可以直接拦截
            {
                // sprintf(command, "(dash 100 %lf)", ball.dir + ball.dir_change);
                Dash(100, ball.dir + ball.dir_change);
                SendCmd(command);
                return;
            }
        }
        if (GotoPos(sidex, sidey)) // 回到初始化点
        {
            SendCmd(command);
            return;
        }
        // sprintf(command, "(turn %lf)", ball.dir);
        Turn(ball.dir);
        SendCmd(command);
        return;
    }

    /*
    如果球被踢出，则转向踢球的方向。
    如果能看到球且球在自己这里，会根据球的位置和状态进行相应的动作，如停球、射门或传球给队友。
    如果看不到球，则转向球的方向。
    如果球可能超出视野范围，会调整转向角度。
    如果球在自己的区域内，会判断是否需要休息、前往截球或跑向球。
    如果球不在自己的区域内，则转向球的方向。
    */
    if (kicked) // 踢球之后转向踢球方向
    {
        // sprintf(command, "(turn %lf)", kickdir);
        Turn(kickdir);
        SendCmd(command);
        return;
    }
    if (ball.visible && ball_on_me) // 球在自己这里
    {
        if (fabs(ball.dir) < view_angle / 2 && ball.speed > 1)
        {
            kickdir = AddAng(head_to_body_angle, ball.dir);
            // sprintf(command, "(kick %lf %lf)", ball.speed / RATE, kickdir); // 停球
            Kick(ball.speed / RATE, kickdir);
            SendCmd(command);
            return;
        }
        if (CanGoal()) // 判断是否可以射门
        {
            kickdir = goaldir;
            // sprintf(command, "(kick 100 %lf)", kickdir); // 射门
            Kick(100, kickdir);
            SendCmd(command);
            return;
        }
        else if (see_mate_num) // 看一下能否传球
        {
            int matei = 0;
            double matex = x;
            if (side == 0) // 左方球队寻找最靠右的队友
            {
                for (int i = 0; i < see_mate_num; i++)
                {
                    if (matex < teammates[i].x)
                    {
                        bool judge = 1; // 判断是否可以传球
                        for (int j = 0; j < see_oppo_num; j++)
                        {
                            if (fabs(SubAng(teammates[i].dir, opponents[j].dir)) < 5)
                            {
                                judge = 0;
                            }
                        }
                        if (judge)
                        {
                            matex = teammates[i].x;
                            matei = i;
                        }
                    }
                }
            }
            else
            {
                for (int i = 0; i < see_mate_num; i++)
                {
                    if (matex > teammates[i].x)
                    {
                        bool judge = 1;
                        for (int j = 0; j < see_oppo_num; j++)
                        {
                            if (fabs(SubAng(teammates[i].dir, opponents[j].dir)) < 5)
                            {
                                judge = 0;
                            }
                        }
                        if (judge)
                        {
                            matex = teammates[i].x;
                            matei = i;
                        }
                    }
                }
            }
            if (fabs(matex - x) < 5) // 不考虑传球，依然射门
            {
                CanGoal();
                kickdir = goaldir;
                // sprintf(command, "(kick 100 %lf)", kickdir);
                Kick(100, kickdir);
                SendCmd(command);
                return;
            }
            kickdir = teammates[matei].dir;
            // sprintf(command, "(kick 100 %lf)", kickdir);
            Kick(100, kickdir);
            SendCmd(command);
            return;
        }
        else // 没有看见队友，依然射门
        {
            CanGoal();
            kickdir = goaldir;
            // sprintf(command, "(kick 100 %lf)", kickdir);
            Kick(100, kickdir);
            SendCmd(command);
            return;
        }
    }
    else
    {
        if (x < minx || x > maxx || y < miny || y > maxy) // 超出了活动范围
        {
            if (turn(SubAng(CalDir(x, y, sidex, sidey), body_global_angle)))
            {
                SendCmd(command);
                return;
            }
            else if (GotoPos(sidex, sidey))
            {
                SendCmd(command);
                return;
            }
            else
                return;
        }
        if (!ball.visible)
        {
            // sprintf(command, "(turn %lf)", (ball.dir > 0) ? view_angle : -view_angle); // 找球
            Turn((ball.dir > 0) ? view_angle : -view_angle);
            SendCmd(command);
            return;
        }
        else if (fabs(ball.dir + ball.dir_change) > view_angle / 2) // 球会超出视野范围
        {
            // sprintf(command, "(turn %lf)", ball.dir + (ball.dir_change > 0 ? view_angle / 2 : -view_angle / 2));
            Turn(ball.dir + (ball.dir_change > 0 ? view_angle / 2 : -view_angle / 2));
            SendCmd(command);
            return;
        }
        else if (BallINField())
        {
            if (stamina < 3000)
            {
                wait = 50; // 体力小于3000，休息50个周期
            }
            if (wait && !BallINOppopenalty())
            {
                wait--;
                return;
            }
            if (BallWay())
            {
                if (GotoPos(get_ball_x, get_ball_y))
                {
                    SendCmd(command); // 前往截球
                    return;
                }
                else
                    return;
            }
            else
            {
                // sprintf(command, "(dash 100 %lf)", ball.dir + ball.dir_change);
                Dash(100, ball.dir + ball.dir_change);
                SendCmd(command); // 跑向球
                return;
            }
        }
        else
        {
            // sprintf(command, "(turn %lf)", ball.dir + ball.dir_change);
            Turn(ball.dir + ball.dir_change);
            SendCmd(command);
            return;
        }
    }
    return;
}
void Client::Run()
{
    if (role == goalkeeper) // 守门员活动范围限定在禁区内
    {
        minx = -52.5;
        maxx = -36;
        miny = -20.16;
        maxy = 20.16;
    }
    else if (role == defender)
    {
        minx = -53;
        maxx = -15;
        miny = -35;
        maxy = 35;
    }
    else if (role == centre_forword)
    {
        minx = -53;
        maxx = 25;
        miny = -35;
        maxy = 35;
    }
    else if (role == striker)
    {
        minx = -15;
        maxx = 53;
        miny = -35;
        maxy = 35;
    }
    else
        return;
    if (role == goalkeeper)
        sprintf(command, "(init %s (version 9) (goalie))", TEAM_NAME); // 守门员上场
    else
        sprintf(command, "(init %s (version 9))", TEAM_NAME); // 普通球员上场
    if (SendCmd(command) == 0)
        return;
    MessageLoop();
}
int main(int argc, char **argv)
{
    if (signal(SIGINT, &SigExitHandle) == SIG_ERR ||
        signal(SIGTERM, &SigExitHandle) == SIG_ERR ||
        signal(SIGHUP, &SigExitHandle) == SIG_ERR)
    {
        cerr << __FILE__ << ": " << __LINE__ << ": could not set signal handler: " << strerror(errno) << endl;
        exit(EXIT_FAILURE);
    }
    string server = "localhost";
    int port = 6000;
    for (int i = 0; i < argc; i++)
    {
        if (strcmp(argv[i], "role") == 0)
            role = (enum PlayerRole)atoi(argv[++i]);
        if (strcmp(argv[i], "inix") == 0)
            inix = (double)atof(argv[++i]);
        if (strcmp(argv[i], "iniy") == 0)
            iniy = (double)atof(argv[++i]);
    }
    client = new Client(server, port);
    client->Run();
    return EXIT_SUCCESS;
}