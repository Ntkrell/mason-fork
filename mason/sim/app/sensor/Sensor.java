package sim.app.sensor;

import sim.engine.*;
import sim.field.continuous.*;
import sim.util.*;

public class Sensor extends SimState {

	double mixing;
	double lrange;
	double srange;
	double threshold;

	int num_nodes;
	double width, height;

	double maxval = 100;

	Continuous2D field, LRField;

	Node[] nodes;

	double average, pwrSumAvg, avg;

	int LRRadioUsedCount = 0;
	int totalLRRadioUsedCount = 0;

	public Sensor(long seed) {
		this(seed, 200, 200, 150, 25, 100, 0.5, 0.01);
	}

	public Sensor(long seed, double width, double height, double lrange, double srange, int num_nodes, double mixing, double thratio) {
		super(seed);
		this.width = width;
		this.height = height;
		this.lrange = lrange;
		this.srange = srange;
		this.num_nodes = num_nodes;
		this.mixing = mixing;
		this.threshold = maxval * thratio;

		field = new Continuous2D(srange / 1.5, width, height);
		LRField = new Continuous2D(lrange / 1.5, width, height);
	}

	private Double2D randLoc() {
		double x = random.nextDouble() * width;
		double y = random.nextDouble() * height;
		return new Double2D(x, y);
	}

	private void createNodes(int n) {
		nodes = new Node[n];
		for (int i = 0; i < n; i++) {
			nodes[i] = new Node(i, random.nextDouble() * maxval, randLoc(), this);
			field.setObjectLocation(nodes[i], nodes[i].loc);
		}

		for (int i = 0; i < n; i++) {
			while (field.getNeighborsExactlyWithinDistance(nodes[i].loc, srange, false).size() == 0) {
				nodes[i].loc = randLoc();
				field.setObjectLocation(nodes[i], nodes[i].loc);
			}
		}

		this.average = this.updateAverage();
	}

	private void createNodesGridRandom(int n, double r) {
		int d = (int)Math.ceil(Math.sqrt(n));
		double cw = width / (d + 1), ch = height / (d + 1);

		nodes = new Node[n];

		int k = 0;
		for (int i = 1; i < d + 1; i++) {
			for (int j = 1; j < d + 1; j++) {
				if (k == n)
					break;

				double x = (random.nextDouble() - 0.5) * cw * r + i * cw;
				double y = (random.nextDouble() - 0.5) * ch * r + j * ch;

				nodes[k] = new Node(k, random.nextDouble() * maxval, new Double2D(x, y), this);
				System.out.printf("Node %d (%g, %g)\n", k, x, y);
				field.setObjectLocation(nodes[k], nodes[k].loc);
				k++;
			}
		}

		this.average = this.updateAverage();
	}

	public double updateAverage() {
		double sum = 0;

		for (int i = 0; i < num_nodes; i++)
			sum += nodes[i].val;

		return sum / num_nodes;
	}

	public void checkComplete() {
		double d = meanDispl();
		System.out.printf("Steps %4d\tMeanDispl %6.3f\tExpect %.3f\tAverage %.3f\tDiff %.3f%%\n",
		                  schedule.getSteps(), d, average, updateAverage(), Math.abs(updateAverage() - average) / average * 100);
		if (d / average < 0.001) {
			int steps = (int)schedule.getSteps();
			System.out.printf("Completed %5d steps Mean LRRadio Ratio %6.3f%%\n", steps, (double)totalLRRadioUsedCount / steps / num_nodes * 100);
			this.kill();
		}
	}

	public double meanDispl() {
		double sum = 0;
		double avg = updateAverage();
		for (int i = 0; i < num_nodes; i++)
			sum += Math.abs(nodes[i].val - avg);
		return sum / num_nodes;
	}

	public void start() {
		//createNodes(num_nodes);
		createNodesGridRandom(num_nodes, 0.25);
		for (int i = 0; i < num_nodes; i++)
			schedule.scheduleRepeating(Schedule.EPOCH, 0, nodes[i], 1);

		schedule.scheduleRepeating(Schedule.EPOCH, 1, new isComplete(), 1);
	}

	public static void main(String[] args) {
		doLoop(Sensor.class, args);
		//System.exit(0);
	}

	private class isComplete implements Steppable {
		private static final long serialVersionUID = 1;

		public void step(SimState state) {
			Sensor s = (Sensor)state;
			s.checkComplete();
			System.out.printf("Step %d LRRadioUsedCount %d\n", s.schedule.getSteps(), LRRadioUsedCount);
			totalLRRadioUsedCount += LRRadioUsedCount;
			s.LRRadioUsedCount = 0;
		}
	}
}