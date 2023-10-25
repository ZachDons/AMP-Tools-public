#include "MyHW6.h"
#include "AMPCore.h"

amp::Path2D MyPointWFAlgo::planInCSpace(const Eigen::Vector2d &q_init, const Eigen::Vector2d &q_goal, const amp::GridCSpace2D &grid_cspace)
{
    init_cell = ZachTools::getCellFromPoint(grid_cspace, q_init.x(), q_init.y());
    goal_cell = ZachTools::getCellFromPoint(grid_cspace, q_goal.x(), q_goal.y()); // initialize path object
    LOG("START CELL: "<<init_cell.first<<"."<<init_cell.second);
    LOG("GOAL CELL: " << goal_cell.first << "," << goal_cell.second);

    DenseArray2D<MyElem> wave_grid(grid_cspace.size().first, grid_cspace.size().second);
    makeMyGridSpace(grid_cspace, wave_grid);

    MyElem last_cell;
    last_cell.index = init_cell;
    MyElem first_cell;
    first_cell.value = 2;
    first_cell.index = goal_cell;
    cell_stack = {first_cell};
    
    int cnt = 0;
    int break_cnt = 10000;
    goal_is_reached = false;
    MyElem cell;
    while (!goal_is_reached)
    {
        cell = cell_stack.front();            // grab first element
        cell_stack.erase(cell_stack.begin()); // remove first element
        expandGridSpace(wave_grid, cell);
        if (cnt > break_cnt){
            LOG("While Loop break (goal reach)...");
            break;
        }
        cnt++;
    }
    LOG("HOLY SHIT IT RAN");

    amp::Path2D path = compilePath(grid_cspace, wave_grid);
    LOG("PATH LENGTH: "<<path.length());
    return path;
}

amp::Path2D MyPointWFAlgo::compilePath(const amp::GridCSpace2D &grid_cspace, const DenseArray2D<MyElem> &wave_grid)
{
    amp::Path2D path;
    std::pair<int, int> current_pos_idx = wave_grid(init_cell.first, init_cell.second).index;
    Vector2d current_pos = ZachTools::getPointFromIndex(grid_cspace, current_pos_idx);
    path.waypoints.push_back(current_pos);
    LOG("Points pushed: ");
    LOG(current_pos_idx.first<<","<<current_pos_idx.second);
    std::pair<int, int> next_pos_idx;
    int cnt = 0;
    int break_cnt = 5000;
    while (!((current_pos_idx.first == goal_cell.first) && (current_pos_idx.second == goal_cell.second)))
    {
        next_pos_idx = wave_grid(current_pos_idx.first, current_pos_idx.second).parent;
        Vector2d next_pos = ZachTools::getPointFromIndex(grid_cspace, next_pos_idx);
        path.waypoints.push_back(next_pos);
        current_pos_idx = next_pos_idx;
        LOG(current_pos_idx.first<<","<<current_pos_idx.second);
        if (cnt > break_cnt){
            LOG("While Loop break (push path)...");
            break;
        }
        cnt++;
    }
    return path;
}

void MyPointWFAlgo::expandGridSpace(DenseArray2D<MyElem> &wave_grid, const MyElem cell)
{
    std::pair<int, int> grid_size = wave_grid.size();
    int a = cell.index.first;
    int b = cell.index.second;
    vector<std::pair<int, int>> neighbor = {{a - 1, b}, {a + 1, b}, {a, b - 1}, {a, b + 1}};
    int num_neighbors = neighbor.size();
    std::pair<int, int> neighbor_pair;
    MyElem n_cell;
    for (int nn = 0; nn < num_neighbors; nn++)
    {
        neighbor_pair = neighbor[nn];

        if (neighbor_pair.first >= 0 && neighbor_pair.first < grid_size.first && neighbor_pair.second >= 0 && neighbor_pair.second < grid_size.second)
        {
            //LOG("Neighbor: " << neighbor_pair.first << "," << neighbor_pair.second);
            if (wave_grid(neighbor_pair.first, neighbor_pair.second).value == 0)
            {
                n_cell.value = cell.value + 1;
                n_cell.index = {neighbor_pair.first, neighbor_pair.second};
                n_cell.parent = {cell.index.first, cell.index.second};
                wave_grid(neighbor_pair.first, neighbor_pair.second) = n_cell;
                cell_stack.push_back(n_cell);
                if ((neighbor_pair.first == init_cell.first) && (neighbor_pair.second == init_cell.second))
                {
                    goal_is_reached = true;
                    break;
                }
            }
        }
    }
}

void MyPointWFAlgo::makeMyGridSpace(const amp::GridCSpace2D &grid_cspace, DenseArray2D<MyElem> &wave_grid)
{
    std::pair<int, int> grid_size = grid_cspace.size();
    int cspace_value;
    for (int i_row = 0; i_row < grid_size.first; i_row++)
    {
        for (int j_col = 0; j_col < grid_size.second; j_col++)
        {
            cspace_value = grid_cspace.operator()(i_row, j_col); // Copy c-space values
            wave_grid(i_row, j_col).value = cspace_value;
            bufferObstacles(grid_cspace, wave_grid, i_row, j_col);
        }
    }
}

void MyPointWFAlgo::bufferObstacles(const amp::GridCSpace2D &grid_cspace, DenseArray2D<MyElem> &wave_grid, const int i_row, const int j_col)
{
    std::pair<int, int> grid_size = grid_cspace.size();
    vector<std::pair<int, int>> neighbor = {{i_row - 1, j_col}, {i_row + 1, j_col}, {i_row, j_col - 1}, {i_row, j_col + 1},{i_row + 1, j_col + 1},{i_row - 1, j_col - 1}};
    int num_neighbors = neighbor.size();
    std::pair<int, int> neighbor_pair;

    for (int nn = 0; nn < num_neighbors; nn++)
    {
        neighbor_pair = neighbor[nn];
        if (neighbor_pair.first >= 0 && neighbor_pair.first < grid_size.first && neighbor_pair.second >= 0 && neighbor_pair.second < grid_size.second)
        {
            if ((grid_cspace.operator()(neighbor_pair.first, neighbor_pair.second)))
            {
                wave_grid(i_row, j_col).value = 1;
                break;
            }
        }
    }
}
