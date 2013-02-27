/*
 * tGame.h
 *
 * This file is part of the Evolution of Swarming project.
 *
 * Copyright 2012 Randal S. Olson, Arend Hintze.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#ifndef _tGame_h_included_
#define _tGame_h_included_

#include "globalConst.h"
#include "tAgent.h"
#include <vector>
#include <map>
#include <set>
#include <string>

using namespace std;

class tOctuplet{
public:
    vector<int> data;
    void loadOctuplet(FILE *f);
};

class tExperiment{
public:
    vector<tOctuplet> dropSequences,sizeSequences,selfSequences;
    vector<vector<vector<bool> > > shouldHit;
    void loadExperiment(char *filename);
    void showExperimentProtokoll(void);
    int drops(void);
    int sizes(void);
    int selves(void);
};

class tGame{
public:
    tExperiment theExperiment;
    void loadExperiment(char *filename);
    string executeGame(tAgent* swarmAgent, tAgent* predatorAgent, FILE *data_file, bool report, double safetyDist, double predatorVisionAngle, int killDelay, double confusionMultiplier);
    tGame();
    ~tGame();
    double calcDistanceSquared(double fromX, double fromY, double toX, double toY);
    double calcAngle(double fromX, double fromY, double fromAngle, double toX, double toY);
    void calcSwarmCenter(double preyX[], double preyY[], bool preyDead[], double& preyCenterX, double& preyCenterY);
    void recalcPredDistTable(double preyX[], double preyY[], bool preyDead[],
                             double predX, double predY,
                             double predDists[swarmSize]);
    void recalcPredAndPreyDistTable(double preyX[], double preyY[], bool preyDead[],
                                    double predX, double predY,
                                    double predDists[swarmSize], double preyDists[swarmSize][swarmSize]);
    void applyBoundary(double& positionVal);
    double sum(vector<double> values);
    double average(vector<double> values);
    double variance(vector<double> values);
    double mutualInformation(vector<int> A,vector<int>B);
    double ei(vector<int> A,vector<int> B,int theMask);
    double computeAtomicPhi(vector<int>A,int states);
    double predictiveI(vector<int>A);
    double nonPredictiveI(vector<int>A);
    double predictNextInput(vector<int>A);
    double computeR(vector<vector<int> > table,int howFarBack);
    double computeOldR(vector<vector<int> > table);
    double entropy(vector<int> list);
    int neuronsConnectedToPreyRetina(tAgent *agent);
    int neuronsConnectedToPredatorRetina(tAgent* agent);

};
#endif
