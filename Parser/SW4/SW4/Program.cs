using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;

namespace SW4
{
    class MainClass
    {
        List<string> lines = new List<string>();
        List<string> mapRobots = new List<string>();
        List<List<string>> mapRobotsSep = new List<List<string>>();
        List<List<string>> mapPolygons = new List<List<string>>();
        List<List<string>> mapPolygonsVertex = new List<List<string>>();
        List<List<List<string>>> mapPaths = new List<List<List<string>>>();
        List<string> mapRandValues = new List<string>();
        double randMargin = 1.1;

        List<double> mapLinearTime = new List<double>();

        public MainClass(string fileCheck)
        {
            ProcessData();
            List<string> linesBP = lines;
            List<string> mapRobotsBP = mapRobots;
            List<List<string>> mapRobotsSepBP = mapRobotsSep;
            List<List<string>> mapPolygonsVertexBP = mapPolygonsVertex;
            List<List<string>> mapPolygonsBP = mapPolygons;
            List<List<List<string>>> mapPathsBP = mapPaths;
            List<string> mapRandValuesBP = mapRandValues;
        }

        public MainClass(){
            ProcessData();
            RandValues();
            Console.WriteLine("    Generating Path");
            pathGenerator_GreedyPassiveTime();
        }

        public void ProcessData()
        {
            var reader = new StreamReader(Environment.CurrentDirectory + "/position.txt");
            var positions = reader.ReadToEnd();
            Console.WriteLine("Collecting and Processing Data");
            lines = positions.Split('\n').ToList();
            Console.WriteLine("    Seperating Map");
            foreach (string line in lines)
            {
                int counter = 0;
                string robot = "";
                for (counter = 0; counter < line.Length; counter++)
                {
                    if (line[counter] == '#')
                    {
                        break;
                    }
                    robot += line[counter];
                }
                mapRobots.Add(robot);
                var polygons = new List<string>();
                var polygon = "";
                for (int x = counter + 1; x < line.Length; x++)
                {
                    if (line[x] == ';')
                    {
                        polygons.Add(polygon);
                        polygon = "";
                        continue;
                    }
                    polygon += line[x];
                }
                if (polygon != "")
                {
                    polygons.Add(polygon);
                }
                mapPolygons.Add(polygons);
            }
            Console.WriteLine("    Separating Robots");
            foreach (string line in mapRobots)
            {
                var robots = new List<string>();
                string robot = "";
                for (int x = 0; x < line.Length; x++)
                {
                    robot += line[x];
                    if (line[x] == ')')
                    {
                        x++;
                        robots.Add(robot);
                        robot = "";
                        continue;
                    }
                }
                mapRobotsSep.Add(robots);
            }
            Console.WriteLine("    Separating Polygons into Vertex");
            foreach (List<string> polygons in mapPolygons)
            {
                var vertices = new List<string>();
                foreach (string polygon in polygons)
                {
                    string vertex = "";
                    for (int x = 0; x < polygon.Length; x++)
                    {
                        vertex += polygon[x];
                        if (polygon[x] == ')')
                        {
                            x++;
                            vertices.Add(vertex);
                            vertex = "";
                            continue;
                        }
                    }
                }
                mapPolygonsVertex.Add(vertices);
            }
        }

        private void RandValues()
        {
            var mapPoints = convertStringToCoMapRobotSep(mapRobotsSep);

            Console.WriteLine("    Separating Polygons");
            for (int x = 0; x < lines.Count; x++)
            {
                //List<string> vertices = new List<string>();
                foreach (string polygon in mapPolygons[x])
                {
                    var vertices = new List<string>();
                    vertices = polygon.Split(',').ToList();
                    foreach (string vertex in vertices)
                    {
                        mapPoints[x].Add(convertStringToCo(vertex));
                    }
                }
            }

            Console.WriteLine("    Calculating Rand Values");
            foreach (List<CO> map in mapPoints)
            {
                double highest = map[0].x;
                double lowest = map[0].x;
                foreach (CO point in map)
                {
                    if (point.x > highest)
                        highest = point.x;
                    else if (point.x < lowest)
                        lowest = point.x;
                    if (point.y > highest)
                        highest = point.y;
                    else if (point.y < lowest)
                        lowest = point.y;
                }
                if (highest > 0)
                    highest = Math.Ceiling(highest * randMargin);
                else if ((int)highest == 0)
                    highest = -1;
                else
                    highest = Math.Floor(highest * randMargin);
                if (lowest > 0)
                    lowest = Math.Ceiling(lowest * randMargin);
                else if ((int)lowest == 0)
                    lowest = -1;
                else
                    lowest = Math.Floor(lowest * randMargin);
                mapRandValues.Add("(" + lowest + "," + highest + ")");
            }
        }

        public int power(int baseNum, int powerNum)
        {
            int total = 1;
            for (int x = 0; x < powerNum; x++)
            {
                total = total * baseNum;
            }
            return total;
        }

        public int indexOfShortest(CO robot, List<CO> availRobots)
        {
            var distances = new List<double>();
            foreach (CO otherRobot in availRobots)
            {
                distances.Add(distBetweenTwoPoints(robot, otherRobot));
            }
            int smallestIndex = 0;
            double smallestValue = distances[0];
            for (int x = 1; x < availRobots.Count; x++)
            {
                if (distances[x] < smallestValue)
                {
                    smallestIndex = x;
                    smallestValue = distances[x];
                }
            }
            return smallestIndex;
        }

        public double distBetweenTwoPoints(CO a, CO b)
        {
            double deltaX = a.x - b.x;
            double deltaY = a.y - b.y;
            return Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));
        }

        public CO convertStringToCo(string point)
        {
            var coPoint = new CO();
            string value = "";
            foreach (char letter in point)
            {
                switch (letter)
                {
                    case '(':
                    case ')':
                        continue;
                    case ',':
                        coPoint.x = Convert.ToDouble(value);
                        value = "";
                        break;
                    default:
                        value += letter;
                        break;
                }
            }
            coPoint.y = Convert.ToDouble(value);
            return coPoint;
        }

        public List<CO> convertListOfStringToCo(List<string> points)
        {
            var result = new List<CO>();
            foreach (string point in points)
            {
                result.Add(convertStringToCo(point));
            }
            return result;
        }

        public List<List<CO>> convertStringToCoMapRobotSep(List<List<string>> paths)
        {
            var coPaths = new List<List<CO>>();
            foreach (List<string> path in paths)
            {
                var coPath = new List<CO>();
                foreach (string point in path)
                {
                    coPath.Add(convertStringToCo(point));
                }
                coPaths.Add(coPath);
            }
            return coPaths;
        }

        static int MyRound(double d)
        {
            if (d < 0)
            {
                return (int)(d - 0.5);
            }
            return (int)(d + 0.5);
        }

        //Greedy Active iterates through a list of active robots, for each finds the closest passive robot in linear distance and goes to it
        //Optimisation - Not really optimised, just to connect up the roots
        public void pathGenerator_GreedyActive()
        {
            for (int x = 0; x < lines.Count; x++)
            {
                Console.WriteLine("         Map: " + (x + 1));
                var paths = new List<List<string>>();
                var points = new List<string>();
                var activeRobots = new List<string>();
                activeRobots.Add(mapRobotsSep[x][0]);
                points.Add(mapRobotsSep[x][0]);
                paths.Add(points);
                mapRobotsSep[x].RemoveAt(0);
                int round = 1;
                while (mapRobotsSep[x].Count != 0)
                {
                    var limit = power(2, round);
                    for (int y = 0; y < limit; y++)
                    {
                        if (mapRobotsSep[x].Count != 0)
                        {
                            points = new List<string>();
                            var nextRobotIndex = indexOfShortest(convertStringToCo(activeRobots[y]), convertListOfStringToCo(mapRobotsSep[x]));
                            activeRobots.Add(mapRobotsSep[x][nextRobotIndex]);

                            points.Add(mapRobotsSep[x][nextRobotIndex]);
                            paths.Add(points);
                            paths[y].Add(mapRobotsSep[x][nextRobotIndex]);
                            int totalNumberofRobotsLeft = mapRobotsSep[x].Count;
                            mapRobotsSep[x].RemoveAt(nextRobotIndex);
                        }
                        else {
                            break;
                        }
                    }
                    round++;
                }
                for (int y = paths.Count - 1; y >= 0; y--)
                {
                    if (paths[y].Count == 1)
                    {
                        paths.RemoveAt(y);
                    }
                }
                mapPaths.Add(paths);
            }
        }

        public class PassiveRobots_GP
        {
            public string point { get; set; }
            public int closestRobot { get; set; }
            public double distance { get; set; }
        }

        //Greedy Passive iterates through the list of passive/inactive robots, and then puts links an active robot and a passive where the distance is the smallest
        //Optimisation - for the total distance that the robot/s after to travel, assuming no obstacles
        public void pathGenerator_GreedyPassive()
        {
            double maxLinearDistance = 10000;
            for (int x = 0; x < lines.Count; x++)
            {
                Console.WriteLine("         Map: " + (x + 1));

                //Generating list of passive robots
                var passiveRobots = new List<PassiveRobots_GP>();
                foreach (string robots in mapRobotsSep[x])
                {
                    var pRobot = new PassiveRobots_GP();
                    pRobot.point = robots;
                    pRobot.closestRobot = -1;
                    pRobot.distance = maxLinearDistance;
                    passiveRobots.Add(pRobot);
                }

                //Setting up values for the first iterations
                List<List<string>> paths = new List<List<string>>();
                List<string> points = new List<string>();
                List<string> activeRobots = new List<string>();
                activeRobots.Add(passiveRobots[0].point);
                points.Add(passiveRobots[0].point);
                paths.Add(points);
                passiveRobots.RemoveAt(0);
                foreach (PassiveRobots_GP pRobot in passiveRobots)
                {
                    pRobot.closestRobot = 0;
                    pRobot.distance = distBetweenTwoPoints(convertStringToCo(activeRobots[0]),convertStringToCo(pRobot.point));
                }

                //Start the algorithm
                while (passiveRobots.Count != 0)
                {
                    double shortestDistance = passiveRobots[0].distance;
                    int passiveRobotSelect = 0;

                    //Finds the shortest distance
                    for (int pRNumber = 1; pRNumber < passiveRobots.Count; pRNumber++)
                    {
                        if (passiveRobots[pRNumber].distance < shortestDistance)
                        {
                            shortestDistance = passiveRobots[pRNumber].distance;
                            passiveRobotSelect = pRNumber;
                        }
                    }

                    //Selected passive robot
                    PassiveRobots_GP spRobot = passiveRobots[passiveRobotSelect];

                    //Adds robot to active robots list
                    activeRobots.Add(spRobot.point);

                    //Create a new path for the robot
                    points = new List<string>();
                    points.Add(spRobot.point);
                    paths.Add(points);

                    //Adds robot to the closests active robot's path
                    paths[spRobot.closestRobot].Add(spRobot.point);

                    //Remove robot from passive robots list
                    passiveRobots.RemoveAt(passiveRobotSelect);

                    activeRobots[spRobot.closestRobot] = spRobot.point;

                    //Update list
                    foreach (PassiveRobots_GP pRobot in passiveRobots)
                    {
                        double distanceFromNewActive = distBetweenTwoPoints(convertStringToCo(spRobot.point), convertStringToCo(pRobot.point));
                        if (pRobot.distance > distanceFromNewActive)
                        {
                            pRobot.closestRobot = activeRobots.Count - 1;
                            pRobot.distance = distanceFromNewActive;
                        }
                    }
                }

                for (int y = paths.Count - 1; y >= 0; y--)
                {
                    if (paths[y].Count == 1)
                    {
                        paths.RemoveAt(y);
                    }
                }
                mapPaths.Add(paths);
                //List<List<List<string>>> mapPathsBP = mapPaths;
            }
        }

        public class ActiveRobots_GPS
        {
            public string point { get; set; }
            public double totalTraveled { get; set; }
        }

        public class ActiveRobots_GPS_DataModel
        {
            public int passiveIndex { get; set; }
            public double distance { get; set; }
        }

        public void pathGenerator_GreedyPassiveTime()
        {
            for (int x = 25; x < 26; x++)
            {
                Console.WriteLine("         Map: " + (x + 1));

                //Generating list of passive robots
                List<string> passiveRobots = new List<string>();
                foreach (string robot in mapRobotsSep[x]){
                    passiveRobots.Add(robot);
                }

                //Setting up values for the first iterations
                List<List<string>> paths = new List<List<string>>();
                List<string> points = new List<string>();
                List<ActiveRobots_GPS> aRobots = new List<ActiveRobots_GPS>();
                ActiveRobots_GPS firstBot = new ActiveRobots_GPS();
                firstBot.point = passiveRobots[0];
                firstBot.totalTraveled = 0;
                aRobots.Add(firstBot);

                points.Add(passiveRobots[0]);
                paths.Add(points);
                passiveRobots.RemoveAt(0);

                //Start the algorithm
                while (passiveRobots.Count != 0)
                {
                    
                    var shortestDistPerARobot = new List<ActiveRobots_GPS_DataModel>();
                    Console.WriteLine("              Passive Robots Left: " + passiveRobots.Count);
                    //For the closest node for each active robot
                    foreach (ActiveRobots_GPS robot in aRobots)
                    {
                        var shortestDist = new ActiveRobots_GPS_DataModel();
                        shortestDist.distance = 10000;
                        for (int y = 0; y < passiveRobots.Count; y++)
                        {
                            double distance = distBetweenTwoPoints(convertStringToCo(robot.point), convertStringToCo(passiveRobots[y]));
                            if (shortestDist.distance > distance)
                            {
                                shortestDist.distance = distance;
                                shortestDist.passiveIndex = y;
                            }
                        }
                        shortestDistPerARobot.Add(shortestDist);
                    }

                    //Calculates the total distance
                    var totalDistance = new List<double>();
                    for (int y = 0; y < aRobots.Count; y++)
                    {
                        totalDistance.Add(aRobots[y].totalTraveled + shortestDistPerARobot[y].distance);
                    }

                    //Finds the shortest total length
                    double shortestTotal = 10000;
                    int aRobotIndex = 0;
                    for (int y = 0; y < aRobots.Count; y++)
                    {
                        if (shortestTotal > totalDistance[y])
                        {
                            shortestTotal = totalDistance[y];
                            aRobotIndex = y;
                        }
                    }

                    aRobots[aRobotIndex].totalTraveled += shortestDistPerARobot[aRobotIndex].distance;

                    var newARobot = new ActiveRobots_GPS();
                    newARobot.totalTraveled = aRobots[aRobotIndex].totalTraveled;
                    newARobot.point = passiveRobots[shortestDistPerARobot[aRobotIndex].passiveIndex];
                    aRobots.Add(newARobot);

                    //Create a new path for the robot
                    points = new List<string>();
                    points.Add(passiveRobots[shortestDistPerARobot[aRobotIndex].passiveIndex]);
                    paths.Add(points);

                    //Adds robot to the closests active robot's path
                    paths[aRobotIndex].Add(passiveRobots[shortestDistPerARobot[aRobotIndex].passiveIndex]);

                    //Change position of selected active robot
                    aRobots[aRobotIndex].point = passiveRobots[shortestDistPerARobot[aRobotIndex].passiveIndex];

                    //Remove robot from passive robots list
                    passiveRobots.RemoveAt(shortestDistPerARobot[aRobotIndex].passiveIndex);
                }

                for (int y = paths.Count - 1; y >= 0; y--)
                {
                    if (paths[y].Count == 1)
                    {
                        paths.RemoveAt(y);
                    }
                }
                mapPaths.Add(paths);
                List<List<List<string>>> mapPathsBP = mapPaths;
            }
        }

        public void smoPyBreak(int mapNumber)
        {
            //List<string> linesBP = lines;
            //List<string> mapRobotsBP = mapRobots;
            //List<List<string>> mapRobotsSepBP = mapRobotsSep;
            //List<List<string>> mapPolygonsBP = mapPolygons;
            //List<List<List<string>>> mapPathsBP = mapPaths;
            //List<string> mapRandValuesBP = mapRandValues;

            
            int z = 1;
            foreach (List<string> path in mapPaths[0])
            {
				Console.WriteLine("     Map: " + mapNumber);
				string polygonContent = "[";
				foreach (string polygon in mapPolygons[mapNumber - 1])
				{
					polygonContent += "[" + polygon + "],";
				}
				if (mapPolygons[mapNumber - 1].Count != 0)
					polygonContent = polygonContent.Remove(polygonContent.Length - 1);
				polygonContent += "]";

				var reader = new StreamReader(Environment.CurrentDirectory + "/rrt.py");
				string pybeginning = reader.ReadToEnd();
				List<string> pyBeginningList = pybeginning.Split('\n').ToList();

				string content = "";
				List<string> result = new List<string>();
				//result.Add("# Linear Time: " + mapLinearTime[mapNumber - 1]);
				result = pyBeginningList;
				content = "obstacleList = " + polygonContent;
				result.Add(content);
				result.Add("rand = " + mapRandValues[mapNumber - 1]);
				result.Add("\ncontent = \"\"");
				result.Add("starttime = datetime.datetime.now()");
				content = "";
                result.Add("print \"Path " + z + " of " + mapPaths[0].Count + "\"");
                result.Add("path = []");
                for (int y = 0; y < path.Count - 1; y++)
                {
                    result.Add("start = " + path[y]);
                    result.Add("goal = " + path[y + 1]);
                    result.Add("print \"     Node " + (y + 1) + " and " + (y + 2) + " of " + path.Count + "\"");
                    if (y == 0)
                    {
                        result.Add("path += rrtpath(obstacleList,start,goal,rand)");
                    }
                    else
                    {
                        result.Add("path += rrtpath(obstacleList,start,goal,rand)[1:]");
                    }
                }
                result.Add("pathStr = str(path)[1:-1] + \";\"");
                result.Add("pathStr = pathStr.replace(\"[\", \"(\")");
                result.Add("pathStr = pathStr.replace(\"]\", \")\")");
                result.Add("f = open('smo2sol-" + mapNumber + "-path-" + (z) + ".txt', 'a+')");
                result.Add("f.write(pathStr)");
                result.Add("f.close");
				File.WriteAllLines(Environment.CurrentDirectory + "/smo2-" + mapNumber + "-path-" + z + ".py", result);
				z++;
            }
            //result.Add("endtime = datetime.datetime.now()");
            //result.Add("timeTaken = endtime - starttime");
            //result.Add("tts = str(timeTaken)");
            //result.Add("content = \"Time Taken: \" + tts + \"\\n\" + content");
            //result.Add("content = content[:-1]");
            //result.Add("#plt.axis('scaled')");
            //result.Add("#plt.grid(True)");
            //result.Add("#plt.pause(0.01)  # Need for Mac");
            //result.Add("#plt.show()");
            //File.WriteAllLines(Environment.CurrentDirectory + "/smo2-" + mapNumber + "-path-" + +".py", result);
        }

        public void smoPy(int mapNumber)
        {
            //List<string> linesBP = lines;
            //List<string> mapRobotsBP = mapRobots;
            //List<List<string>> mapRobotsSepBP = mapRobotsSep;
            //List<List<string>> mapPolygonsBP = mapPolygons;
            //List<List<List<string>>> mapPathsBP = mapPaths;
            //List<string> mapRandValuesBP = mapRandValues;

            Console.WriteLine("     Map: " + mapNumber);
            string polygonContent = "[";
            foreach (string polygon in mapPolygons[mapNumber - 1])
            {
                polygonContent += "[" + polygon + "],";
            }
            if (mapPolygons[mapNumber - 1].Count != 0)
                polygonContent = polygonContent.Remove(polygonContent.Length - 1);
            polygonContent += "]";

            var reader = new StreamReader(Environment.CurrentDirectory + "/rrt.py");
            string pybeginning = reader.ReadToEnd();
            List<string> pyBeginningList = pybeginning.Split('\n').ToList();

            string content = "";
            List<string> result = new List<string>();
            //result.Add("# Linear Time: " + mapLinearTime[mapNumber - 1]);
            result = pyBeginningList;
            content = "obstacleList = " + polygonContent;
            result.Add(content);
            result.Add("rand = " + mapRandValues[mapNumber - 1]);
            result.Add("\ncontent = \"\"");
            result.Add("starttime = datetime.datetime.now()");
            content = "";
            int z = 1;
            foreach (List<string> path in mapPaths[mapNumber - 1])
            {
                result.Add("print \"Path " + z++ + " of " + mapPaths[mapNumber - 1].Count + "\"");
                result.Add("path = []");
                for (int y = 0; y < path.Count - 1; y++)
                {
                    result.Add("start = " + path[y]);
                    result.Add("goal = " + path[y + 1]);
                    result.Add("print \"     Node " + (y + 1) + " and " + (y + 2) + " of " + path.Count + "\"");
                    if (y == 0)
                    {
                        result.Add("path += rrtpath(obstacleList,start,goal,rand)");
                    }
                    else {
                        result.Add("path += rrtpath(obstacleList,start,goal,rand)[1:]");
                    }
                }
                result.Add("pathStr = str(path)[1:-1] + \";\"");
                result.Add("pathStr = pathStr.replace(\"[\", \"(\")");
                result.Add("pathStr = pathStr.replace(\"]\", \")\")");
                result.Add("content += pathStr");
            }
            result.Add("endtime = datetime.datetime.now()");
            result.Add("timeTaken = endtime - starttime");
            result.Add("tts = str(timeTaken)");
            result.Add("content = \"Time Taken: \" + tts + \"\\n\" + content");
            result.Add("content = content[:-1]");
            result.Add("f = open('smo2sol-" + mapNumber + ".txt', 'w')");
            result.Add("f.write(content)");
            result.Add("f.close\n");
            result.Add("#plt.axis('scaled')");
            result.Add("#plt.grid(True)");
            result.Add("#plt.pause(0.01)  # Need for Mac");
            result.Add("#plt.show()");
            File.WriteAllLines(Environment.CurrentDirectory + "/smo2-" + mapNumber + ".py", result);
        }

        public static void Main(string[] args)
        {
            MainClass test = new MainClass();
            //test.createMaps();
            //Console.WriteLine("Generating Map");
            for (int x = 1; x <= 1; x++)
            {
                test.smoPyBreak(26);
            }
            //Console.WriteLine(MyRound(-2.4343423424));
            //Console.ReadLine();
            //test.AIODistPy(18 , 18);
            //test.AIODistPy(1, 30);
            //CO first = new CO();
            //CO second = new CO();
            //first.x = 1d;
            //first.y = 1d;
            //second.x = 2d;
            //second.y = 2d;
            //Console.WriteLine(test.distBetweenTwoPoints(first, second));
            //Console.WriteLine(test.power(2,3));
        }
    }
}
