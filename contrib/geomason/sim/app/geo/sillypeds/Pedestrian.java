//***************************************************************
//Copyright 2011 Center for Social Complexity, GMU
//
//Author: Andrew Crooks and Sarah Wise, GMU
//
//Contact: acrooks2@gmu.edu & swise5@gmu.edu
//
//
//sillypeds is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.
//
//It is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//
//***************************************************************
package sim.app.geo.sillypeds;

import java.util.ArrayList;
import java.util.Random;

import sim.engine.SimState;
import sim.engine.Steppable;
import sim.engine.Stoppable;
import sim.util.Bag;



/** 
 *  A Steppable object that updates its position on the landscape as a function
 *  of its local cost surface.
 */
class Pedestrian implements Steppable
{

    SillyPeds world;		// the simulation of which the Pedestrian is a part
    Tile tile; 			// the Basin in which the Pedestrian currently resides
    Space space;
    Stoppable stopper;		// a variable used to unschedule the Pedestrian
    private static final long serialVersionUID = 1L;



    /**
     * Constructor function.
     * @param ww - the WaterWorld object, kept so the Pedestrian can update the
     * 		simulation if it exits the simulation
     * @param l - the ObjectGrid2D landscape object, kept so the Pedestrian can
     * 		determine its local cost surface
     * @param t - the Basin in which the Pedestrian finds itself
     */
    public Pedestrian(SillyPeds ww, Space s, Tile t)
    {
        world = ww;
        space = s;
        tile = t;
        stopper = null;
    }



    /**
     * Steppable that moves the Pedestrian across the landscape.
     */
    @Override
    public void step(SimState state)
    {

        // if on an exit, move to the next space
        if (tile.exit)
        {

            Entrance e = space.exit(tile);

            if (e == null)
            {
                // exiting the simulation
                tile.removePed(this);
                world.peds.remove(this);
                return;
            } else if (e.entrance.peds.size() > 0)
            {
                // exit blocked by another person, can't move through it. Stay in place,
                // but make sure to reschedule self to run again!
                state.schedule.scheduleOnce(this, (int) (1 + tile.baseheight));
                return;
            }

            // leave old space, enter new space
            tile.removePed(this);
            space = e.space;
            tile = e.entrance;
            tile.addPed(this);
        } else
        {
            // get a copy of all of the neighbors of this tile
            Bag neighbors = new Bag();
            space.field.getNeighborsMaxDistance(
                tile.loc_x, tile.loc_y, 1, false, neighbors, null, null);

            // find the set of neighbors that is of minimal height
            ArrayList<Tile> mins = new ArrayList<Tile>();
            double currentheight = tile.baseheight;

            for (Object o : neighbors)
            {
                Tile b = (Tile) o;
                if (b.peds.size() > 0)
                {
                    continue; // the tile is occupied, can't move there
                } else if (b.baseheight <= currentheight)
                {
                    mins.add(b); // add our new find to it
                }
            }

            if (mins.size() > 0)
            { // somewhere is both free and desirable

                // select randomly from the eligible neighbors
                Tile newbasin = mins.get(state.random.nextInt(mins.size()));

                // move to this new spot
                tile.removePed(this);
                newbasin.addPed(this);
                tile = newbasin;
            }
        }

        // schedule self to update next turn!
        // order of update is proportional to distance from exit, e.g. gradient of the tile
        state.schedule.scheduleOnce(this, (int) (1 + tile.baseheight));
    }

}