/*
 * File: cellconversion.h
 * Author: Stewart Nash
 * Description: Converts wall locations to grid coordinates for
 * statespace.h. Code should be extended and made more flexible.
 */

/*
 Wall 13
 Wall 15
 Wall 2
 Wall 20
 Wall 3
 Wall 4
 Wall 5
*/

double wallSize[][3] = {
        {20, 0.15, 2.5},
        {10, 0.15, 2.5},
        {30, 0.15, 2.5},
        {10, 0.15, 2.5},
        {18, 0.15, 2.5},
        {30, 0.15, 2.5},
        {18 ,0.15, 2.5}
};

double wallLocalLocation[][3] = {
        {0, 0, 1.25},
        {0, 0, 1.25},
        {0, 0, 1.25},
        {0, 0, 1.25},
        {0, 0, 1.25},
        {0, 0, 1.25},
        {0, 0, 1.25}
};

double wallLocalAngle[][3] = {
        {0, -0, 0},
        {0, -0, 0},
        {0, -0, 0},
        {0, -0, 0},
        {0, -0, 0},
        {0, -0, 0},
        {0, -0, 0}
};

double wallLocation[][3] = {
        {0, 0, 0},
        {-10, -2, 0},
        {0, -11, 0},
        {10, -2, 0},
        {15, -2, 0},
        {0, 7, 0},
        {-15, -2, 0},
};

double wallAngles[][3] = {
        {0, -0, 0},
        {0, 0, -1.5708},
        {0, -0, 0},
        {0, 0, -1.56904},
        {0, -0, 1.5708},
        {0, -0, 3.14159},
        {0, 0, -1.5708}
};

