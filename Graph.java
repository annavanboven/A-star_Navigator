import java.util.ArrayList;
import java.util.*;
/**
 * This graph class creates an undirected weighted graph, whose vertices and edges are both objects. It is made to 
 * hold vertices that have a latitude and longitude. This graph also uses the A* algorithm to calculate the shortest 
 * path between two vertices. The heurustic used is the distance between two points on a sphere, where the two points
 * are two locations on the globe.
 *
 * @author Anna Van Boven
 * @version November 20, 2020
 * I worked with SaraJane on this project.
 */
public class Graph 
{
    public int numVertices; //counts the number of vertices added to the array list 
    public int numEdges; //counts the number of edges added to the graph
    public Vertex[] vertices; //an array that holds every vertex in the graph
    private final int R = 6371; //radius of earth
    public HashMap<String, Integer> cities;   //in this hash map, the key is the city name, and the value is its 
                                              //location in the vertex array

    public Graph(int size){
        vertices = new Vertex[size]; //set the array to be the size of the number of edges
        cities = new HashMap<>();
        numVertices = 0;
        numEdges = 0;
    }

    /**
     * The Edge class holds an Edge object, which will store the weight of the edge and the two vertices that 
     * connect it.
     * 
     */
    private class Edge implements Comparable<Edge>{
        public Vertex v1;
        public Vertex v2;
        double w;

        Edge(Vertex vert1, Vertex vert2, double weight){
            v1 = vert1;
            v2 = vert2;
            w = weight;  
        }

        /**
         * Returns the weight of the edge
         */
        public double getWeight(){
            return w;
        }

        /**
         * Returns whatever vertex isn't given that shares the same edge.
         * 
         * @param v     the initial vertex
         * @return      the other vertex
         */
        public Vertex getOther(Vertex v){
            if(v.city.equals(v1.city)){
                return v2;
            }
            else if (v.city.equals(v2.city)){
                return v1;
            }
            else{
                return null;
            }
        }

        /**
         * Returns one of the vertices for the edge.
         */
        public Vertex getEither(){
            return v1;
        }

        /**
         * The compareTo method compares the weight of the current edge to the weight of another edge. If the current 
         * edge is bigger, it returns 1. If the current edge is smaller, it returns -1. If the edges are the same
         * weight, it returns 0.
         */
        @Override
        public int compareTo(Edge other){
            double temp = w-other.w;
            if(temp > 0){
                temp = 1;
            }
            else if (temp < 0){
                temp = -1;
            }
            return (int)temp;
        }

        /**
         * The toString() method prints general information about the edge
         * 
         * @return  a string containing the two vertices and the weight
         */
        public String toString(){
            String s = "v1 = " + v1.city + ", v2 = " + v2.city + ", weight = " + w;
            return s;
        }
    }

   /**
    * The Vertex class holds a vertex object, which will hold the latitude and longitude of the vertex along with
    * the name of the city itself.
    * 
    * @param lat    the latitude
    * @param llong  the longitude
    * @param name   the city name
    * 
    */
    class Vertex implements Comparable<Vertex>{
        public double latitude; //latitude
        public double longitude; //longitude
        public ArrayList<Edge> edges; //stores an arraylist of all the edges connected to this vertex
        public double score; //holds the score of the vertex when come across in a search
        public double gx; //holds the distance travelled to this vertex when come across in a search
        public String city; //city name
        public Vertex prev; //the vertex that led to this vertex in a search
        Vertex(double lat, double llong, String name){
            latitude = lat;
            longitude = llong;
            edges = new ArrayList<Edge>();
            score = 0;
            gx = 0;
            city = name;
            prev = null;
        }

        /**
         * Returns a list of the edges connected to this vertex.
         * 
         * @return  edge ArrayList
         */
        public ArrayList<Edge> getEdges(){
            return edges;
        }

        /**
         * @return  the city name
         */
        public String toString(){
            return city;
        }

        /**
         * The Heur method takes in a vertex and calculates the distance between it and the current vertex using
         * the formula for distance between two points on a sphere.
         * 
         * @param v     the other vertex
         * @return      the crow-flies distance between current vertex and the other vertex
         */
        public double heur(Vertex v){
            //convert all the values from degrees to radians
            double lata = this.latitude*(Math.PI/180);
            double latb = v.latitude*(Math.PI/180);
            double lona = this.longitude*(Math.PI/180);
            double lonb = v.longitude*(Math.PI/180);
            //uses distance formula to calculate the distance
            double dist = Math.acos(Math.sin(lata)*Math.sin(latb)+Math.cos(lata)*Math.cos(latb)*Math.cos(lona-lonb))*R;
            return dist;
        }

        /**
         * The recalcScore method calculates the score that the current vertex has in a search by adding its 
         * distance to the heuristic.
         * 
         * @param goal  the end goal for the search
         */
        public void recalcScore(Vertex goal){
            score = setDist() + heur(goal);
        }

        /**
         * setPrev sets the current vertex's prev value 
         * @param v     the vertex's previous value
         */
        public void setPrev(Vertex v){
            prev = v;
        }

        /**
         * The setDist method calculates the distance travelled in a search to reach the current vertex.
         * @return distance travelled
         */
        public double setDist(){
            if(prev==null){
                gx=0;
            }
            else{
                gx = prev.gx+sharedEdge(prev).w;
            }
            return gx;
        }

        /**
         * the compareTo method compares two vertices based on their current scores, with the smaller score receiving
         * a value of 1.
         * 
         * @param v     the other vertex
         * @return      the position of the current vertex in comparison to the other vertex
         */
        @Override
        public int compareTo(Vertex v){
            if(this.score > v.score){
                return -1;
            }
            else if(this.score < v.score){
                return 1;
            }
            else{
                return 0;
            }
        }

        /**
         * The sharedEdge method finds the edge shared between two vertices.
         * @param v     the other vertex
         * @return      the shared edge
         */
        public Edge sharedEdge(Vertex v){
            Edge edge = null;
            //loops through each edge connected to the current vertex, checking to see if the edge
            //connects the current vertex with the inputted vertex
            for(int i = 0; i < edges.size(); i++){
                if(edges.get(i).getOther(this).city.equals(v.city)){
                    edge = edges.get(i);
                }
            }
            return edge;
        }
    }

    /**
     * The addVertex method adds a vertex to the graph.
     * 
     * @param v1    name of the city
     * @param lat   latitude
     * @param lon   longitude
     */
    public void addVertex(String v1,double lat, double lon){
        cities.put(v1,numVertices); //put into the hashmap to store its index in the vertices array
        Vertex vert = new Vertex(lat, lon,v1); //create a new vertex
        vertices[numVertices] = vert; //store it in the array
        numVertices++;
    }

    /**
     * Returns the number of vertices, which is incremented every time a vertex is added.
     * 
     * @return numVertices
     */
    public int countVertices(){
        return numVertices;
    }

    /**
     * Returns the number of edges, which is incremented every time an edge is added.
     * 
     * @return numEdges
     */
    public int countEdges(){
        return numEdges;
    }

    /**
     * The addEdge method calls another method that adds an edge bewteen two vertices.
     * 
     * @param v1        first vertex
     * @param v2        second vertex
     * @param weight    edge weight
     */
    public void addEdge(String v1, String v2, double weight){
        //gets the position of the vertices in the array
        int ve1 = cities.get(v1); 
        int ve2 = cities.get(v2);
        //calls the method on two vertices instead of two strings
        addEdge(vertices[ve1],vertices[ve2],weight);
        //increments
        numEdges++;
    }

    /**
     * The addEdge mathod adds an edge between two vertices.
     * 
     * @param v1        first vertex
     * @param v2        second vertex
     * @param weight    edge weight
     */
    private void addEdge(Vertex v1, Vertex v2, double weight){
        //creates a new edge
        Edge edge = new Edge(v1,v2,weight);
        //adds the new edge to the list of edges that each vertex stores
        v1.edges.add(edge);
        v2.edges.add(edge);
    }

    /**
     * The getScore method returns the score of a vertex in a search.
     * 
     * @param v1    vertex to search
     * @return score
     */
    public double getScore(String v1){
        int v = cities.get(v1);
        return vertices[v].score;
    }

    /**
     * The areAdjacent method returns whether two vertices share an edge.
     * 
     * @param v1,v2 the two vertices
     * @return  true if they share an edge, false if otherwise
     */
    private boolean areAdjacent(Vertex v1, Vertex v2){
        for(int i = 0; i < v1.edges.size(); i++){
            if(v1.edges.get(i).getOther(v1).city.equals(v2.city)){
                return true;
            }
        }
        return false;
    }

    /**
     * The getAdjacencyList returns a list of all the vertices that are adjacent to a vertex.
     * 
     * @param v1    vertex to explore
     * @return      array of adjacent vertices
     */
    public Vertex[] getAdjacencyList(Vertex v1){
        Vertex[] adj = new Vertex[v1.edges.size()];
        //loops through all the edges connected to the vertex and adds the vertex at theother end of the edge to the
        //array
        for(int i = 0; i < v1.edges.size(); i++){
            adj[i] = v1.edges.get(i).getOther(v1);
        }
        return adj;
    }

    /**
     * The heur method takes in two city names and calculates the heuristic between the first and the second.
     * 
     * @param v1    initial vertex
     * @param v2    goal vertex
     * @return      heuristic distance
     */
    public double heur(String v1, String v2){
        int ve1 = cities.get(v1);
        int ve2 = cities.get(v2);
        return heur(vertices[ve1],vertices[ve2]);
    }

    /**
     * The heur method takes in two vertices and calculates the heuristic between the first and the second.
     * 
     * @param v1    initial vertex
     * @param v2    goal vertex
     * @return      heuristic distance
     */
    private double heur(Vertex v1, Vertex v2){        
        return v1.heur(v2);
    }

    /**
     * The astar method takes in two city names and calculates the shortest path between them.
     * 
     * @param v1    start vertex name
     * @param v2    goal vertex name
     * @return      An arraylist of the path taken to connect the vertices
     */
    public ArrayList<Vertex> astar(String v1, String v2){
        //gets the vertices from the list and calls a helper method
        int ve1 = cities.get(v1);
        int ve2 = cities.get(v2);
        return astar(vertices[ve1],vertices[ve2]);
    }

    /**
     * The astar method takes in two city names and calculates the shortest path between them using the distance
     * travelled and a heuristic. 
     * 
     * @param v1    start vertex
     * @param v2    goal vertex
     * @return      An arraylist of the path taken to connect the vertices
     */
    private ArrayList<Vertex> astar(Vertex start, Vertex end){
        PriorityQueue<Vertex> next = new PriorityQueue<Vertex>(); //open list of vertices to explore
        //The hashtable stores vertices already explored as the key, with the value being the vertex that led to
        //that key
        Hashtable<Vertex,Vertex> closed = new Hashtable<Vertex,Vertex>(); 
        boolean reachGoal = false;
        Vertex curr = start;
        //loops through all vertices connected to the start vertex
        for(int i = 0; i < start.edges.size(); i++){
            Edge e = start.edges.get(i);
            Vertex v = e.getOther(start);
            v.recalcScore(end); //calculates score
            v.setPrev(start);  //sets previous vertex
            next.add(v); //adds to priority queue
        }

        closed.put(start,start); //puts the first vertex into the closed list

        while(next.size()>0 && !reachGoal){ //while the priority queue isn't empty and the goal hasn't been reached
            curr = next.poll();  //take the next item from the priority queue
            if(curr.city.equals(end.city)){ //if you've reached the goal
                reachGoal = true;
                closed.put(curr,curr.prev); //put on closed list in order to correctly trace back the path
                curr.recalcScore(end); //get the final score
                break;
            }

            //for every vertex connected to the current vertex
            for(int i = 0; i < curr.edges.size(); i++){
                Edge e = curr.edges.get(i);
                Vertex v = e.getOther(curr);
                v.recalcScore(end);
                //if it is not already in the pq and it hasn't been explored yet
                if(!next.contains(v) && !closed.containsKey(v)){ 
                    v.setPrev(curr);
                    next.add(v);
                }
            }

            //put current on the closed list
            closed.put(curr,curr.prev);
        }
        
        //if you haven't met your goal and the priority queue is empty, there is no path to connect the two.
        if(!reachGoal){
            return null;
        }
        return returnPath(closed,start,end);
    }

    
    /**
     * The returnPath method retraces the path taken by a search and returns the final path. This method will
     * never be called if there is not a guaranteed path
     * 
     * @param finder    hashtable storing a list of vertices and where those vertices came from
     * @param start     the initial vertex given in the search
     * @param end       the goal vertex of the search
     * @return path     an arraylist of the path taken from the start node to the goal node
     */
    private ArrayList<Vertex> returnPath(Hashtable<Vertex,Vertex> finder, Vertex start, Vertex end){
        ArrayList<Vertex> path = new ArrayList<Vertex>();
        Vertex curr = end;
        Vertex from;
        while(!curr.city.equals(start.city)){ //while the start isn't reached, retrace
            from = finder.get(curr);
            path.add(0,curr);
            curr=from;            
        }
        //add the first step onto the path
        path.add(0,curr);
        return path;
     }
     
     /**
      * The reset method resets values in the vertices between searches.
      * 
      */
     public void reset(){
         //loops through every vertex
         for(int i = 0; i<numVertices; i++){
            vertices[i].score = 0;
            vertices[i].gx = 0;
            vertices[i].prev=null;
        }
        }

    /**
     * The print method prints a graph by vertex, along with the edges for each vertex.
     */
    public void print(){
        for(int i = 0; i < numVertices; i++){
            System.out.println(vertices[i].toString());
            for(int j = 0; j< vertices[i].edges.size(); j++){
                System.out.println(vertices[i].edges.get(j).toString());
            }
        }
    }
}
