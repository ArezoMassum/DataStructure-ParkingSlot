package algotwo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Scanner;

class Slot { // slot class to represent each parking slot
    int index; //slot number
    int capacity; //number of vehicles it can hold
    int cost;
    int taken; //number of vehicles currently parked

    public Slot(int index, int capacity, int cost) {
        this.index = index;
        this.capacity = capacity;
        this.cost = cost;
        this.taken = 0;
    }
}

class Edge { //class to represent connections between slots
    int v; // destination slot
    int w; //weight

    public Edge(int v, int w) {
        this.v = v;
        this.w = w;
    }
}

class Main {
    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in); //read the input values

        int N = scanner.nextInt(); // Number of slots
        int M = scanner.nextInt(); // Number of edges
        int F = scanner.nextInt(); // Base parking fee

        List<Slot> slots = new ArrayList<>(); //list to store information about each slot
        for (int i = 0; i < N; i++) {
            int capacity = scanner.nextInt();
            slots.add(new Slot(i, capacity, F));
        }

        List<List<Edge>> graph = new ArrayList<>(); //list to represent connections between the slots
        for (int i = 0; i < N; i++) {
            graph.add(new ArrayList<>());
        }

        for (int i = 0; i < M; i++) { //reading the edges connecting the slots 
            int u = scanner.nextInt();
            int v = scanner.nextInt();
            int w = scanner.nextInt();
            //updating the graph accordingly
            graph.get(u - 1).add(new Edge(v - 1, w));
            graph.get(v - 1).add(new Edge(u - 1, w)); // Add reverse edge for undirected graph
        }

        int K = scanner.nextInt(); //  reading number of vehicles

        int[] costs = new int[K]; //array to store the minimum cost for each vehicle
        Arrays.fill(costs, -1); // Initialize costs to -1

        for (int k = 0; k < K; k++) { //iterate over each vehicle from 0 to k-1
            int minimumCost = Integer.MAX_VALUE; //Initialising minimumCost as Integer.Max_VALUE to keep track of the minimum cost for the current vehicle
            int selectedSlot = -1; //initialising selectedSlot as -1 to store the index of the slot with the min cost for the curr vehicle.

            //Dijkstra algorithm to find min cost path from the entrance slot(0)
            int[] dist = new int[N];
            Arrays.fill(dist, Integer.MAX_VALUE); //current min distance from  entrance to each slot 
            dist[0] = 0; //distance from entrance to itself

            PriorityQueue<Edge> pq = new PriorityQueue<>((a, b) -> a.w - b.w); //storing edges and weights(weight in ascending order
            pq.offer(new Edge(0, 0)); //add the entrance slot to the pq

            while (!pq.isEmpty()) { //while not empty
                Edge edge = pq.poll(); //pop the edge with min weight
                int u = edge.v;
                int w = edge.w;

                if (w > dist[u]) { //if weight of popped edge is greater than the current min distance to next slot, 
                    continue; //continue iteration
                }

                Slot slot = slots.get(u);
                
                //checking on slots if capacity of current is greater than the number of vehicles already parked in
                if (slot.capacity > slot.taken) {
                    int cost = slot.cost + w; //calculating cost for current vehicle(base cost+ path cost)

                    if (cost < minimumCost) { //if calculated cost is less than the current cost, update the min cost
                        minimumCost = cost;
                        selectedSlot = u; //store the index of selected slot
                    }
                }

                for (Edge neighbor : graph.get(u)) { //iterating over neighbours of current vertex
                    int v = neighbor.v;
                    int weight = neighbor.w;
                    //if new distance is smaller than the current min distance 
                    if (dist[u] + weight < dist[v]) {
                        dist[v] = dist[u] + weight; //update distance 
                        pq.offer(new Edge(v, dist[v])); //add neighbour to the pq
                    }
                }
            }
//if slot was selected for the current vehicle,  
            if (selectedSlot != -1) {
                costs[k] = minimumCost;  //update the costs array with min cost
                Slot selected = slots.get(selectedSlot);
                selected.taken++; //increment the occupied number of selected slot
            }
        }

        for (int i = 0; i < K; i++) { //print the cost for each vehicle in the costs array
            System.out.print(costs[i] + " ");
        }
    }
}