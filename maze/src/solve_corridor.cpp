#include <a_star.h>
#include <iostream>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <maze.h>

using namespace std;
using namespace ecn;

// a node is a x-y position that already knows its children
class Position : public ecn::Point
{
    typedef std::unique_ptr<Position> PositionPtr;

public:

    Position(int _x, int _y, int d) : Point(_x, _y), dist(d) {}
    Position(ecn::Point p) : Point(p.x, p.y) {}

    void setStartGoal(Position start, Position goal)
    {
        x_start = start.x;
        y_start = start.y;
        x_goal = goal.x;
        y_goal = goal.y;
    }

    int distToParent()
    {
        return dist;
    }

    bool is_corridor(int _x, int _y, int &i, int &j)
    {
        if(i && maze.isFree(_x+i, _y) +
                maze.isFree(_x, _y+1) +
                maze.isFree(_x, _y-1) == 1)
        {
            if(maze.isFree(_x, _y-1))
            {
                i = 0;
                j = -1;
            }
            else if(maze.isFree(_x, _y+1))
            {
                i = 0;
                j = 1;
            }
            return true;
        }
        else if(j && maze.isFree(_x, _y+j) +
                maze.isFree(_x+1, _y) +
                maze.isFree(_x-1, _y) == 1)
        {
            if(maze.isFree(_x-1, _y))
            {
                i = -1;
                j = 0;
            }
            else if(maze.isFree(_x+1, _y))
            {
                i = 1;
                j = 0;
            }
            return true;
        }
        return false;
    }

    void print(Point parent)
    {
        if(&parent)
        {
            // look for path to parent
            std::vector<std::pair<int, int>> path;
            for(const auto &dir: vector<Pair>({{-1,0},{1,0},{0,-1},{0,1}}))
            {
                path.clear();
                int i(dir.first);
                int j(dir.second);
                int x_cur(x+i), y_cur(y+j);
                int k(0);

                if(maze.isFree(x_cur, y_cur))
                {
                    path.push_back({x_cur, y_cur});
                    k = 1;
                    while(is_corridor(x_cur, y_cur, i, j)
                          && (x_cur != x_start || y_cur != y_start))
                    {
                        x_cur += i;
                        y_cur += j;
                        path.push_back({x_cur, y_cur});
                        k++;
                    }

                    if(x_cur == parent.x && y_cur == parent.y)
                    {
                        for(auto p = path.rbegin(); p!=path.rend(); p++)
                            maze.passThrough(p->first, p->second);
                        break;
                    }
                }
            }
        }
        maze.passThrough(x, y);
    }


//here we reddfine the show function in point class, just switch the position of parameter
    void show( Point parent ,bool closed = false)
    {
        const int b = closed?255:0, r = closed?0:255;
        if(&parent)
        {
            // look for path to parent
            std::vector<std::pair<int, int>> path;
            path.reserve(10);
            for(const auto &dir: vector<Pair>({{-1,0},{1,0},{0,-1},{0,1}}))
            {
                path.clear();
                int i(dir.first);
                int j(dir.second);
                int x_cur(x+i), y_cur(y+j);

                // follow corridor till end or crossing
                int k(0);

                if(maze.isFree(x_cur, y_cur))
                {
                    path.push_back({x_cur, y_cur});
                    k = 1;
                    while(is_corridor(x_cur, y_cur, i, j)
                          && (x_cur != x_start || y_cur != y_start))
                    {
                        x_cur += i;
                        y_cur += j;
                        path.push_back({x_cur, y_cur});
                        k++;
                    }

                    if(x_cur == parent.x && y_cur == parent.y)
                    {
                        for(const auto &p: path)
                            maze.write(p.first, p.second, r, 0, b, false);
                        break;
                    }
                }
            }
        }
        maze.write(x, y, r, 0, b);
    }


    std::vector<PositionPtr> children()
    {
        std::vector<PositionPtr> generated;

        for(const auto &dir: vector<Pair>({{-1,0},{1,0},{0,-1},{0,1}}))
        {
            int i(dir.first);
            int j(dir.second);
            int x_cur(x+i), y_cur(y+j);

            // follow corridor till end or crossing
            int k(0);
            if(maze.isFree(x_cur, y_cur))
            {
                k = 1;

                while(is_corridor(x_cur, y_cur, i, j)
                      && (x_cur != x_goal || y_cur != y_goal))
                {
                    x_cur += i;
                    y_cur += j;
                    k++;
                }
                generated.push_back(PositionPtr(new Position(x_cur, y_cur, k)));
            }
        }

        return generated;
    }

protected:
    int dist;
    static int x_goal, y_goal, x_start, y_start;
};

int Position::x_goal, Position::y_goal, Position::x_start, Position::y_start;

typedef std::pair<int, int> Pair;

int main( int argc, char **argv )
{
    std::string filename = "maze.png";
    if(argc == 2)
        filename = std::string(argv[1]);

    // build nodes from maze
    Position::maze = ecn::Maze(filename);

    // initial and goal positions
    Position start = Position::maze.start(),
            goal = Position::maze.end();
    start.setStartGoal(start, goal);
    ecn::Astar(start, goal);

    Position::maze.saveSolution("corridor");
    cv::waitKey(0);
}

