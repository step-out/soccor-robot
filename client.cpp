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

#elif defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
#define RCSS_WIN
#include <winsock2.h>
#endif

#ifndef RCSS_WIN
#include <unistd.h>     
#include <sys/select.h> 
#include <sys/time.h>   
#include <sys/types.h>  
#endif

using namespace std;

enum PlayerRole
{
    goalkeeper,
    defender,
    centre_forword,
    striker
};

const double PI = 3.14159265359;           
const char TEAM_NAME[20] = "soccor-eater"; 
const double MARGIN = 0.7;                 
const double RATE = 0.027;
const double PLAYER_SIZE = 0.3; 
const double BALL_SIZE = 0.085; 
const double MAX_SPEED = 1;     
const double DASH_POWER_RATE = 0.006;


char play_mode[30];    
int current_cycle = 0; 
int last_cycle = 0;    


int id = 1;                
PlayerRole role = striker; 
bool side = 0;             
double inix = -50;    
double iniy = 0;
double sidex = -50; 
double sidey = 0;
double x = -50; 
double y = 0;
bool infield = 1;    
double minx = -52.5; 
double maxx = 52.5;
double miny = -34;
double maxy = 34;
double speed = 0;              
double speed_to_head_dir = 0;  
double speed_global_angle = 0; 
double head_to_body_angle = 0; 
double head_global_angle = 0;  
double body_global_angle = 0;  
double view_angle = 90;        
char view_width[10];           
double stamina = 8000;         
double effort = 1;             
int see_mate_num = 0;          
int see_oppo_num = 0;          
bool ball_on_me = 0;           
bool ball_on_side = 0;         
bool ball_on_opposide = 0;     
bool catch_moved = 0;          
int catch_wait = 0;            
int kick_times = 0;            
int catch_times = 0;           
bool kicked = 0;               
bool catched = 0;              
double kickdir = 0;            
double goaldir, goaldist;      
double goal_x, goal_y;
double goal_tx, goal_ty;
double goal_bx, goal_by;
double get_ball_x = 0; 
double get_ball_y = 0;
int wait = 0;     
bool waited = 0;  
char command[50]; 


struct Ball 
{
    bool visible;              
    bool moving;               
    double dist_change;        
    double dir_change;         
    double x[50];              
    double y[50];              
    double dist;               
    double dir;                
    double speed;              
    double speed_global_angle; 
} ball;


struct Player 
{
    double x;
    double y;
    double dist;
    double dir;
    double dist_change;
    double dir_change;
} teammates[11], opponents[22]; 


struct Flags 
{
    bool visible; 
    double dist;  
    double dir;   
} flags[55];      
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
        "(f t l 10)", "(f t l 20)", "(f t l 30)", "(f t l 40)", "(f t l 50)"}; 
struct Flag_coord
{
    double x;
    double y;
}; 
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


struct Lines 
{
    bool visible;
    double dist;
    double dir;
} lines[4];
char line_name[4][6] = {"(l l)", "(l r)", "(l t)", "(l b)"}; 
double line_global_angle[4] = {-180, 0, -90, 90};            


double AngToRad(double x) 
{
    return x * PI / 180.0;
}
double RadToAng(double x) 
{
    return x * 180.0 / PI;
}
double AddAng(double x, double y) 
{
    if (x + y > -180 && x + y <= 180)
        return x + y;
    else if (x + y <= -180)
        return x + y + 360;
    else
        return x + y - 360;
}
double SubAng(double x, double y) 
{
    if (x - y > -180 && x - y <= 180)
        return x - y;
    else if (x - y <= -180)
        return x - y + 360;
    else
        return x - y - 360;
}
double CalDist(double x1, double y1, double x2, double y2) 
{
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
double CalDir(double x1, double y1, double x2, double y2) 
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
double RandBetween(double a, double b) 
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
    
    bool SendCmd(char *command);
    
    void Init(char *msg);      
    void Hear(char *msg);      
    void SenseBody(char *msg); 
    void See(char *msg);       
    
    void Kick(double power, double direction);
    void Dash(double power, double direction);
    void Turn(double moment);
    void Turn_neck(double moment);
    void Tackle(double power);
    void Move(double x, double y);
    void Catch(double direction);
    void ChangeView(const char *width, const char *quality);
    
    void UpdateINFO();                
    bool turn(double angle);          
    bool GotoPos(double x, double y); 
    bool BallINField();               
    bool BallINPenalty();             
    bool BallINOppopenalty();         
    bool CanGoal();                   
    int BallWay();                    
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
void Client::Init(char *msg) 
{
   
    char Side;
    
    sscanf(msg, "(init %c %d %s)", &Side, &id, play_mode);
    play_mode[strlen(play_mode) - 1] = 0;
    if (Side == 'l') 
    {
        side = 0;
        x = sidex = inix;
        y = sidey = iniy;
        goal_x = flag_coord[3].x; 
        goal_y = flag_coord[3].y;
        goal_tx = flag_coord[4].x;
        goal_ty = flag_coord[4].y;
        goal_bx = flag_coord[5].x;
        goal_by = flag_coord[5].y;
    }
    else 
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
        goal_x = flag_coord[0].x; 
        goal_y = flag_coord[0].y;
        goal_tx = flag_coord[1].x;
        goal_ty = flag_coord[1].y;
        goal_bx = flag_coord[2].x;
        goal_by = flag_coord[2].y;
    }
}
void Client::Hear(char *msg) 
{
    sscanf(msg, "(hear %d referee %s)", &current_cycle, play_mode);
    play_mode[strlen(play_mode) - 1] = 0; 
}
void Client::SenseBody(char *msg) 
{
    int last_kick = kick_times;
    int last_catch = catch_times;
    sscanf(msg, "(sense_body %d (view_mode %*s %s (stamina %lf %lf) (speed %lf %lf) (head_angle %lf) "
                "(kick %d) (dash %*d) (Turn %*d) (say %*d) (turn_neck %*d) (catch %d)",
           &current_cycle, view_width, &stamina, &effort,
           &speed, &speed_to_head_dir, &head_to_body_angle, &kick_times, &catch_times);
    view_width[strlen(view_width) - 1] = 0;

    if (!strcmp(view_width, "wide"))
        view_angle = 180; 
    else if (!strcmp(view_width, "normal"))
        view_angle = 90; 
    else
        view_angle = 45; 

    if (kick_times == last_kick + 1)
        kicked = 1; 
    else
        kicked = 0;

    if (catch_times == last_catch + 1)
        catched = 1; 
}
void Client::See(char *msg) 
{
    sscanf(msg, "(see %d", &current_cycle);
    char *p = 0;
    char format[40];
    char str[30];
    for (int i = 0; i < 55; i++)
    {
        p = strstr(msg, flagName[i]); 
        if (p != 0)
        {
            flags[i].visible = 1; 
            sprintf(format, "%s %%lf %%lf", flagName[i]);
            sscanf(p, format, &flags[i].dist, &flags[i].dir); 
        }
        else
        {
            flags[i].visible = 0; 
        }
    }
    for (int i = 0; i < 4; i++)
    {
        p = strstr(msg, line_name[i]); 
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
    sprintf(str, "(p"); 
    p = strstr(msg, str);
    int mate_i = 0, oppo_i = 0;
    for (int i = 0; p; i++)
    {
        p += 2;
        if (*p == ')') 
        {
            p++;
            sscanf(p, "%lf %lf", &opponents[oppo_i].dist, &opponents[oppo_i].dir);
            oppo_i++;
        }
        else
        {
            p += 1; 
            char tmp[50];
            sscanf(p, "%s", tmp);
            if(tmp[0] == '"')
            {
                tmp[strlen(tmp) - 1] = '\0';
                if(tmp[strlen(tmp) - 1] == '"')
                    tmp[strlen(tmp) - 1] = '\0';
            }
            p = strstr(p, ")");
            if (!strcmp(tmp, TEAM_NAME)) 
            {
                teammates[mate_i].dist_change = teammates[mate_i].dir_change = 0;
                sscanf(p, "%*s %lf %lf %lf %lf", &teammates[mate_i].dist, &teammates[mate_i].dir, &teammates[mate_i].dist_change, &teammates[mate_i].dir_change);
                mate_i ++;
            }
            else 
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
    p = strstr(msg, str); 
    if (p == 0)
    {
        sprintf(str, "(B)"); 
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
    sprintf(str, "(G)"); 
    p = strstr(msg, str);
    if (p != 0)
    {
        sprintf(format, "%s %%lf %%lf", str);
        if (x < 0) 
        {
            flags[0].visible = 1;
            sscanf(p, format, &flags[0].dist, &flags[0].dir);
        }
        else 
        {
            flags[3].visible = 1;
            sscanf(p, format, &flags[3].dist, &flags[3].dir);
        }
    }
}
void Client::Kick(double power, double direction)
{
   
    sprintf(command, "(kick %lf %lf)", power, direction);
}
void Client::Dash(double power, double direction)
{
   
   
    sprintf(command, "(dash %lf %lf)", power, direction);
}
void Client::Turn(double moment)
{
   
    sprintf(command, "(turn %lf)", moment);
}
void Client::Turn_neck(double moment)
{
    sprintf(command, "(turn_neck %lf)", moment);
}
void Client::Tackle(double power)
{
   
    sprintf(command, "(tackle %lf)", power);
}
void Client::Move(double x, double y)
{
    sprintf(command, "(move %lf %lf)", x, y);
}
void Client::Catch(double direction)
{
   
    sprintf(command, "(catch %lf)", direction);
}
void Client::ChangeView(const char *width, const char *quality)
{
   
    sprintf(command, "(change_view %s %s)", width, quality);
}
void Client::UpdateINFO() 
{
    int seelinenum = 0, j = 0;
    for (int i = 0; i < 4; i++)
    {
        if (lines[i].visible)
        {
            seelinenum++;
            if (seelinenum != 1)
            {
                j = lines[j].dist > lines[i].dist ? j : i; 
                break;
            }
            else
                j = i;
        }
    }
    if (seelinenum != 1)
        infield = 0;                                                                                                  
    head_global_angle = SubAng(line_global_angle[j], (lines[j].dir < 0) ? (lines[j].dir + 90) : (lines[j].dir - 90)); 
    body_global_angle = SubAng(head_global_angle, head_to_body_angle);                                                
    speed_global_angle = AddAng(speed_to_head_dir, head_global_angle);                                                
    double mindist = 1000;
    for (int i = 0; i < 55; i++)
    {
        if (flags[i].visible && flags[i].dist < mindist)
        {
            j = i;
            mindist = flags[i].dist;
        }
    } 
    x = flag_coord[j].x - flags[j].dist * cos(AngToRad(AddAng(flags[j].dir, head_global_angle)));
    y = flag_coord[j].y - flags[j].dist * sin(AngToRad(AddAng(flags[j].dir, head_global_angle)));
    for (int i = 0; i < see_mate_num; i++) 
    {
        teammates[i].x = x + teammates[i].dist * cos(AngToRad(AddAng(teammates[i].dir, head_global_angle)));
        teammates[i].y = y + teammates[i].dist * sin(AngToRad(AddAng(teammates[i].dir, head_global_angle)));
    }
    for (int i = 0; i < see_oppo_num; i++) 
    {
        opponents[i].x = x + opponents[i].dist * cos(AngToRad(AddAng(opponents[i].dir, head_global_angle)));
        opponents[i].y = y + opponents[i].dist * sin(AngToRad(AddAng(opponents[i].dir, head_global_angle)));
    }
    if (ball.visible) 
    {
        ball.x[0] = ball.x[1]; 
        ball.y[0] = ball.y[1];
        ball.x[1] = x + ball.dist * cos(AngToRad(AddAng(ball.dir, head_global_angle))); 
        ball.y[1] = y + ball.dist * sin(AngToRad(AddAng(ball.dir, head_global_angle)));
        if (CalDist(ball.x[0], ball.y[0], ball.x[1], ball.y[1]) < 0.1)
            ball.moving = 0; 
        else
            ball.moving = 1;
        ball.speed_global_angle = CalDir(ball.x[0], ball.y[0], ball.x[1], ball.y[1]); 
        for (int i = 2; i < 50; i++)
        { 
            ball.x[i] = ball.x[i - 1] + CalDist(ball.x[i - 2], ball.y[i - 2], ball.x[i - 1], ball.y[i - 1]) * 0.94 * cos(AngToRad(ball.speed_global_angle));
            ball.y[i] = ball.y[i - 1] + CalDist(ball.x[i - 2], ball.y[i - 2], ball.x[i - 1], ball.y[i - 1]) * 0.94 * sin(AngToRad(ball.speed_global_angle));
        }
        ball.speed = CalDist(ball.x[1], ball.y[1], ball.x[2], ball.y[2]); 
        if (ball.dist <= MARGIN + BALL_SIZE + PLAYER_SIZE)                
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
                    ball_on_opposide = 1; 
                }
            }
            if (!ball_on_opposide)
            {
                for (int i = 0; i < see_mate_num; i++) 
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
bool Client::turn(double angle) 
{
    static int turn_times = 0;
    static double turn_angle = 0;
    if (turn_times == 0)
    {
        turn_angle = angle; 
    }
    if (fabs(turn_angle) < 3) 
    {
        turn_times = 0;
        turn_angle = 0;
        return 0;
    }
    turn_times++;
    double fact_turn = turn_angle / (1.0 + 5.0 * speed); 
    Turn(turn_angle);
    turn_angle = SubAng(turn_angle, fact_turn);
    return 1;
}
bool Client::GotoPos(double x, double y) 
{
    if (CalDist(x, y, x, y) < MARGIN)
        return 0;
    else
    {
        Dash(100, SubAng(CalDir(x, y, x, y), body_global_angle));
        return 1;
    }
}
bool Client::BallINField() 
{
    if (ball.x[1] >= minx && ball.x[1] <= maxx && ball.y[1] >= miny && ball.y[1] <= maxy)
        return 1;
    else
        return 0;
}
bool Client::BallINPenalty() 
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
bool Client::BallINOppopenalty() 
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
bool Client::CanGoal() 
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
int Client::BallWay() 
{
    if (!ball.moving)
        return 0;
    if (CalDist(x, y, ball.x[1], ball.y[1]) <= CalDist(x, y, ball.x[2], ball.y[2]))
    {
        return 0;
    }
    double mindist = ball.dist;
    for (int i = 2; i < 50; i++) 
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
    if (!strncmp(msg, "(init", 5)) 
    {
        Init(msg);
        ChangeView("narrow", "high");
        SendCmd(command);
        return;
    }
    else if (!strncmp(msg, "(hear", 5)) 
    {
        Hear(msg);
    }
    else if (!strncmp(msg, "(sense_body", 11)) 
    {
        SenseBody(msg);
    }
    else if (!strncmp(msg, "(see", 4)) 
    {
        See(msg);
        UpdateINFO(); 
    }
    else
        return;

    if (last_cycle != current_cycle)
    {
        last_cycle = current_cycle; 
        return;
    }

    if (!strcmp(play_mode, "before_kick_off") || !strncmp(play_mode, "goal_l", 6) || !strncmp(play_mode, "goal_r", 6))
    {
        
        if (CalDist(x, y, sidex, sidey) > 1)
        {
            Move(inix, iniy);
            SendCmd(command);
            return;
        }
        else if (turn(SubAng(CalDir(x, y, 0, 0), body_global_angle))) 
        {
            SendCmd(command);
            return;
        }
        else
            return;
    }
    else if (!strcmp(play_mode, "kick_off_l") && side == 0) 
    {
        if (id == 11) 
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
    else if (!strcmp(play_mode, "kick_off_r") && side == 1) 
    {
        if (id == 11) 
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
    { 
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
                        Kick(100, SubAng(side ? 180 : 0, body_global_angle));
                        SendCmd(command);
                        return;
                    }
                }
                else if (!ball.visible)
                {
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
    { 
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
            Turn(view_angle);
            SendCmd(command);
            return;
        }
        else
            return;
    }
    if (role == goalkeeper) 
    {
        if (ball.visible && ball.dist <= 2) 
        {
            if (fabs(ball.dir) > 90)
            {
                Dash(100, ball.dir);
                SendCmd(command);
                return;
            }
            else
            {
                Catch(AddAng(head_to_body_angle, ball.dir));
                SendCmd(command);
                return;
            }
        }
        else if (!ball.visible)
        {
            Turn(view_angle);
            SendCmd(command);
            return;
        }
        else if (fabs(ball.dir + ball.dir_change) > view_angle / 2) 
        {
            Turn(ball.dir + ball.dir_change);
            SendCmd(command);
            return;
        }
        else if (BallINPenalty()) 
        {
            if (ball.speed > 1) 
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
                    Turn(ball.dir);
                    SendCmd(command);
                    return;
                }
            }
            else 
            {
                Dash(100, ball.dir + ball.dir_change);
                SendCmd(command);
                return;
            }
        }
        if (GotoPos(sidex, sidey)) 
        {
            SendCmd(command);
            return;
        }
        Turn(ball.dir);
        SendCmd(command);
        return;
    }

   
    if (kicked) 
    {
        Turn(kickdir);
        SendCmd(command);
        return;
    }
    if (ball.visible && ball_on_me) 
    {
        if (fabs(ball.dir) < view_angle / 2 && ball.speed > 1)
        {
            kickdir = AddAng(head_to_body_angle, ball.dir);
            Kick(ball.speed / RATE, kickdir);
            SendCmd(command);
            return;
        }
        if (CanGoal()) 
        {
            kickdir = goaldir;
            Kick(100, kickdir);
            SendCmd(command);
            return;
        }
        else if (see_mate_num) 
        {
            int matei = 0;
            double matex = x;
            if (side == 0) 
            {
                for (int i = 0; i < see_mate_num; i++)
                {
                    if (matex < teammates[i].x)
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
            if (fabs(matex - x) < 5) 
            {
                CanGoal();
                kickdir = goaldir;
                Kick(100, kickdir);
                SendCmd(command);
                return;
            }
            kickdir = teammates[matei].dir;
            Kick(100, kickdir);
            SendCmd(command);
            return;
        }
        else 
        {
            CanGoal();
            kickdir = goaldir;
            Kick(100, kickdir);
            SendCmd(command);
            return;
        }
    }
    else
    {
        if (x < minx || x > maxx || y < miny || y > maxy) 
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
            Turn((ball.dir > 0) ? view_angle : -view_angle);
            SendCmd(command);
            return;
        }
        else if (fabs(ball.dir + ball.dir_change) > view_angle / 2) 
        {
            Turn(ball.dir + (ball.dir_change > 0 ? view_angle / 2 : -view_angle / 2));
            SendCmd(command);
            return;
        }
        else if (BallINField())
        {
            if (stamina < 3000)
            {
                wait = 50; 
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
                    SendCmd(command); 
                    return;
                }
                else
                    return;
            }
            else
            {
                
                Dash(100, ball.dir + ball.dir_change);
                SendCmd(command); 
                return;
            }
        }
        else
        {
            
            Turn(ball.dir + ball.dir_change);
            SendCmd(command);
            return;
        }
    }
    return;
}
void Client::Run()
{
    if (role == goalkeeper) 
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
        sprintf(command, "(init %s (version 9) (goalie))", TEAM_NAME); 
    else
        sprintf(command, "(init %s (version 9))", TEAM_NAME); 
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