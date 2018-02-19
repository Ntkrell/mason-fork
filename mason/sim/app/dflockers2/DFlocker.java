/*
  Copyright 2006 by Sean Luke and George Mason University
  Licensed under the Academic Free License version 3.0
  See the file "LICENSE" for more information
*/

package sim.app.dflockers2;
import sim.engine.*;
import sim.field.CommAgent;
import sim.field.DObjectMigrator.AgentOutputStream;
import sim.field.continuous.*;
import sim.util.*;

import java.io.IOException;
import java.io.ObjectInputStream;

import ec.util.*;

public class DFlocker extends CommAgent implements Steppable, sim.portrayal.Orientable2D {
    private static final long serialVersionUID = 1;

    public Double2D loc = new Double2D(0, 0);
    public Double2D lastd = new Double2D(0, 0);
    public boolean dead = false;

    public DFlocker(Double2D location) { loc = location;}

    public double getOrientation() { return orientation2D(); }
    public boolean isDead() { return dead; }
    public void setDead(boolean val) { dead = val; }

    public void setOrientation2D(double val) {
        lastd = new Double2D(Math.cos(val), Math.sin(val));
    }

    public double orientation2D() {
        if (lastd.x == 0 && lastd.y == 0) return 0;
        return Math.atan2(lastd.y, lastd.x);
    }

    public Double2D momentum() {
        return lastd;
    }
    
    public  void writePrimitiveTypeData(AgentOutputStream out)
	{		
		try
		{
			out.os.writeBoolean(dead);
			out.os.writeDouble(loc.x);
			out.os.writeDouble(loc.y);
			out.os.writeDouble(lastd.x);
			out.os.writeDouble(lastd.y);
		} catch (IOException e)
		{
			e.printStackTrace();
			System.exit(-1);
		}
	}
    
    public void readPrimitiveTypeData(ObjectInputStream in)
	{
		try
		{
			this.dead = in.readBoolean();
			double x = in.readDouble();
			double y = in.readDouble();
			this.loc = new Double2D(x, y);
			x = in.readDouble();
			y = in.readDouble();
			this.lastd = new Double2D(x, y);
		}
		catch (IOException e)
		{
			e.printStackTrace();
			System.exit(-1);
		}
		
	}
	
	public Object clone()
	{
		DFlocker flocker =  new DFlocker(new Double2D(loc.x, loc.y));
		flocker.setDead(isDead());
		flocker.setOrientation2D(getOrientation());
		flocker.lastd = new Double2D(lastd.x, lastd.y);
		return flocker;
	}

    public Double2D consistency(Bag b, Continuous2D flockers) {
        if (b == null || b.numObjs == 0) return new Double2D(0, 0);

        double x = 0;
        double y = 0;
        int i = 0;
        int count = 0;
        for (i = 0; i < b.numObjs; i++) {
            DFlocker other = (DFlocker)(b.objs[i]);
            if (!other.dead) {
                Double2D m = ((DFlocker)b.objs[i]).momentum();
                count++;
                x += m.x;
                y += m.y;
            }
        }
        if (count > 0) { x /= count; y /= count; }
        return new Double2D(x, y);
    }

    public Double2D cohesion(Bag b, Continuous2D flockers) {
        if (b == null || b.numObjs == 0) return new Double2D(0, 0);

        double x = 0;
        double y = 0;

        int count = 0;
        int i = 0;
        for (i = 0; i < b.numObjs; i++) {
            DFlocker other = (DFlocker)(b.objs[i]);
            if (!other.dead) {
                double dx = flockers.tdx(loc.x, other.loc.x);
                double dy = flockers.tdy(loc.y, other.loc.y);
                count++;
                x += dx;
                y += dy;
            }
        }
        if (count > 0) { x /= count; y /= count; }
        return new Double2D(-x / 10, -y / 10);
    }

    public Double2D avoidance(Bag b, Continuous2D flockers) {
        if (b == null || b.numObjs == 0) return new Double2D(0, 0);
        double x = 0;
        double y = 0;

        int i = 0;
        int count = 0;

        for (i = 0; i < b.numObjs; i++) {
            DFlocker other = (DFlocker)(b.objs[i]);
            if (other != this ) {
                double dx = flockers.tdx(loc.x, other.loc.x);
                double dy = flockers.tdy(loc.y, other.loc.y);
                double lensquared = dx * dx + dy * dy;
                count++;
                x += dx / (lensquared * lensquared + 1);
                y += dy / (lensquared * lensquared + 1);
            }
        }
        if (count > 0) { x /= count; y /= count; }
        return new Double2D(400 * x, 400 * y);
    }

    public Double2D randomness(MersenneTwisterFast r) {
        double x = r.nextDouble() * 2 - 1.0;
        double y = r.nextDouble() * 2 - 1.0;
        double l = Math.sqrt(x * x + y * y);
        return new Double2D(0.05 * x / l, 0.05 * y / l);
    }

    public void step(SimState state) {
        final DFlockers flock = (DFlockers)state;
        Double2D oldloc = loc;
        loc = flock.flockers.getObjectLocation(this);
        if (loc == null) {
            System.out.printf("pid %d oldx %g oldy %g", flock.flockers.p.pid, oldloc.x, oldloc.y);
            Thread.dumpStack();
            System.exit(-1);
        }

        if (dead) return;
        Bag b = flock.flockers.getNeighborsExactlyWithinDistance(loc, flock.neighborhood, true);

        Double2D avoid = avoidance(b, flock.flockers);
        Double2D cohe = cohesion(b, flock.flockers);
        Double2D rand = randomness(flock.random);
        Double2D cons = consistency(b, flock.flockers);
        Double2D mome = momentum();

        double dx = flock.cohesion * cohe.x + flock.avoidance * avoid.x + flock.consistency * cons.x + flock.randomness * rand.x + flock.momentum * mome.x;
        double dy = flock.cohesion * cohe.y + flock.avoidance * avoid.y + flock.consistency * cons.y + flock.randomness * rand.y + flock.momentum * mome.y;

        // renormalize to the given step size
        double dis = Math.sqrt(dx * dx + dy * dy);
        if (dis > 0) {
            dx = dx / dis * flock.jump;
            dy = dy / dis * flock.jump;
        }

        lastd = new Double2D(dx, dy);
        loc = new Double2D(flock.flockers.stx(loc.x + dx), flock.flockers.sty(loc.y + dy));

        flock.flockers.setObjectLocation(this, loc);
    }
}
