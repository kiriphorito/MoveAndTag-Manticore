using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;

namespace SW4
{
    public class PyVisGraphGen
    {
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
            List<double> distances = new List<double>();
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
            CO coPoint = new CO();
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
            List<CO> result = new List<CO>();
            foreach (string point in points)
            {
                result.Add(convertStringToCo(point));
            }
            return result;
        }

        public List<List<CO>> convertStringToCoMapRobotSep(List<List<string>> paths)
        {
            List<List<CO>> coPaths = new List<List<CO>>();
            foreach (List<string> path in paths)
            {
                List<CO> coPath = new List<CO>();
                foreach (string point in path)
                {
                    coPath.Add(convertStringToCo(point));
                }
                coPaths.Add(coPath);
            }
            return coPaths;
        }

        public void IndividualPy()
        {
            var reader = new StreamReader(Environment.CurrentDirectory + "/position.txt");
            string positions = reader.ReadToEnd();
            Console.WriteLine(Environment.CurrentDirectory + "position.txt");
            List<string> lines = positions.Split('\n').ToList();

            List<string> mapRobots = new List<string>();
            List<List<string>> mapPolygons = new List<List<string>>();

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
                List<string> polygons = new List<string>();
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

            List<List<string>> mapRobotsSep = new List<List<string>>();
            foreach (string line in mapRobots)
            {
                List<string> robots = new List<string>();
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

            for (int x = 0; x < 16; x++)
            {
                string content = "";
                content += "import pyvisgraph as vg\n";
                content += "polys = [";
                if (mapPolygons[x].Count != 0)
                {
                    foreach (string polygon in mapPolygons[x])
                    {
                        content += "[";
                        //content += polygon;
                        for (int y = 0; y < polygon.Length; y++)
                        {
                            if (polygon[y] == '(')
                            {
                                content += "vg.Point";
                            }
                            content += polygon[y];
                        }
                        content += "],";
                    }
                    content = content.Remove(content.Length - 1);
                }
                content += "]\n";
                content += "g = vg.VisGraph()\n";
                content += "g.build(polys)\n";
                for (int y = 0; y < mapRobotsSep[x].Count - 1; y++)
                {
                    if (y == 0)
                    {
                        content += "shortest = g.shortest_path(vg.Point" + mapRobotsSep[x][y] + ", vg.Point" + mapRobotsSep[x][y + 1] + ")\n";
                        continue;
                    }
                    content += "shortest += g.shortest_path(vg.Point" + mapRobotsSep[x][y] + ", vg.Point" + mapRobotsSep[x][y + 1] + ")\n";
                }
                content += "print shortest";
                File.WriteAllText(Environment.CurrentDirectory + "/" + x + ".py", content);
            }
        }

        public void AIODistPy(int starting, int ending)
        {
            var reader = new StreamReader(Environment.CurrentDirectory + "/position.txt");
            string positions = reader.ReadToEnd();
            Console.WriteLine(Environment.CurrentDirectory + "position.txt");
            List<string> lines = positions.Split('\n').ToList();

            List<string> mapRobots = new List<string>();
            List<List<string>> mapPolygons = new List<List<string>>();

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
                List<string> polygons = new List<string>();
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

            List<List<string>> mapRobotsSep = new List<List<string>>();
            foreach (string line in mapRobots)
            {
                List<string> robots = new List<string>();
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

            List<List<CO>> CORobotSep = new List<List<CO>>();
            CORobotSep = convertStringToCoMapRobotSep(mapRobotsSep);

            List<List<List<string>>> mapPaths = new List<List<List<string>>>();

            int mapRobotsSepC = mapRobotsSep.Count;

            for (int x = 0; x < lines.Count; x++)
            {
                List<List<string>> paths = new List<List<string>>();
                List<string> points = new List<string>();
                List<string> activeRobots = new List<string>();
                activeRobots.Add(mapRobotsSep[x][0]);
                points.Add(mapRobotsSep[x][0]);
                paths.Add(points);
                mapRobotsSep[x].RemoveAt(0);
                int round = 1;
                while (mapRobotsSep[x].Count != 0)
                {
                    int limit = power(2, round);
                    for (int y = 0; y < limit; y++)
                    {
                        if (mapRobotsSep[x].Count != 0)
                        {
                            points = new List<string>();
                            foreach (string roboto in activeRobots)
                            {
                                Console.WriteLine(roboto);
                            }
                            int nextRobotIndex = indexOfShortest(convertStringToCo(activeRobots[y]), convertListOfStringToCo(mapRobotsSep[x]));
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

            string content = "";
            List<string> result = new List<string>();
            result.Add("import pyvisgraph as vg");
            result.Add("f = open('maticore', 'w')");
            result.Add("fileContentSol = \"\"");
            result.Add("fileContentDM = \"\"");
            result.Add("totalS = \"\"");
            result.Add("totalDM = \"\"");
            for (int x = (starting - 1); x <= (ending - 1); x++)
            {
                content = "";
                Console.WriteLine("Map: " + (x + 1));
                result.Add("print \"Map " + (x + 1) + "\"");
                content += "polys = [";
                if (mapPolygons[x].Count != 0)
                {
                    foreach (string polygon in mapPolygons[x])
                    {
                        content += "[";
                        //content += polygon;
                        for (int y = 0; y < polygon.Length; y++)
                        {
                            if (polygon[y] == '(')
                            {
                                content += "vg.Point";
                            }
                            content += polygon[y];
                        }
                        content += "],";
                    }
                    content = content.Remove(content.Length - 1);
                }
                content += "]";
                result.Add(content);
                result.Add("print \"    Building Map\"");
                result.Add("g = vg.VisGraph()");
                if (mapPolygons[x].Count != 0)
                {
                    result.Add("print \"    Core Count = 6\"");
                    result.Add("g.build(polys, workers=6)");
                }
                else {
                    result.Add("print \"    Core Count = 1\"");
                    result.Add("g.build(polys)");
                }
                result.Add("print \"    Shortest Path\"");
                foreach (List<string> path in mapPaths[x])
                {
                    for (int y = 0; y < path.Count - 1; y++)
                    {
                        if (y == 0)
                        {
                            result.Add("shortest = g.shortest_path(vg.Point" + path[y] + ", vg.Point" + path[y + 1] + ")");
                            continue;
                        }

                        result.Add("shortest += g.shortest_path(vg.Point" + path[y] + ", vg.Point" + path[y + 1] + ")[1:]");
                    }
                    result.Add("s = str(shortest)");
                    result.Add("s = s[1:-1] + \";\"");
                    result.Add("totalS += s");
                    result.Add("s = str(shortest) + \",\"");
                    result.Add("totalDM += s");
                    result.Add("s = \"\"");
                }
                result.Add("totalS = \"" + (x + 1) + ": \" + totalS[:-1] + \"\\n\"");
                result.Add("totalDM = \"" + (x + 1) + ": [\" + totalDM[:-1] + \"]\\n\"");
                result.Add("fileContentSol += totalS");
                result.Add("fileContentDM += totalDM");
                result.Add("totalS = \"\"");
                result.Add("totalDM = \"\"");
            }
            result.Add("fileContent = \"Solution Output\\n\" + fileContentSol + \"\\n\\nDrawMap.py Output\\n\" + fileContentDM");
            result.Add("f.write(fileContent)");
            result.Add("f.close");
            File.WriteAllLines(Environment.CurrentDirectory + "/aio.py", result);
        }

        public void AIOPy(int starting, int ending)
        {
            var reader = new StreamReader(Environment.CurrentDirectory + "/position.txt");
            string positions = reader.ReadToEnd();
            Console.WriteLine(Environment.CurrentDirectory + "position.txt");
            List<string> lines = positions.Split('\n').ToList();

            List<string> mapRobots = new List<string>();
            List<List<string>> mapPolygons = new List<List<string>>();

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
                List<string> polygons = new List<string>();
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

            List<List<string>> mapRobotsSep = new List<List<string>>();
            foreach (string line in mapRobots)
            {
                List<string> robots = new List<string>();
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

            List<List<List<string>>> mapPaths = new List<List<List<string>>>();
            foreach (List<string> availRobots in mapRobotsSep)
            {
                int counter = 0;
                List<List<string>> paths = new List<List<string>>();
                int round = 0;
                while (counter < availRobots.Count)
                {
                    int limit = power(2, round);
                    for (int x = 0; x < limit; x++)
                    {
                        if (counter < availRobots.Count)
                        {
                            List<string> path = new List<string>();
                            path.Add(availRobots[counter]);
                            paths.Add(path);
                            paths[x].Add(availRobots[counter]);
                            counter++;
                        }
                        else {
                            break;
                        }
                    }
                    round++;
                }
                for (int x = paths.Count - 1; x >= 0; x--)
                {
                    if (paths[x].Count == 1)
                    {
                        paths.RemoveAt(x);
                    }
                }
                paths[0].RemoveAt(0);
                mapPaths.Add(paths);
            }

            string content = "";
            List<string> result = new List<string>();
            result.Add("import pyvisgraph as vg");
            result.Add("f = open('maticore', 'w')");
            result.Add("fileContentSol = \"\"");
            result.Add("fileContentDM = \"\"");
            result.Add("totalS = \"\"");
            result.Add("totalDM = \"\"");
            for (int x = (starting - 1); x <= (ending - 1); x++)
            {
                content = "";
                Console.WriteLine("Map: " + (x + 1));
                result.Add("print \"Map " + (x + 1) + "\"");
                content += "polys = [";
                if (mapPolygons[x].Count != 0)
                {
                    foreach (string polygon in mapPolygons[x])
                    {
                        content += "[";
                        //content += polygon;
                        for (int y = 0; y < polygon.Length; y++)
                        {
                            if (polygon[y] == '(')
                            {
                                content += "vg.Point";
                            }
                            content += polygon[y];
                        }
                        content += "],";
                    }
                    content = content.Remove(content.Length - 1);
                }
                content += "]";
                result.Add(content);
                result.Add("print \"    Building Map\"");
                result.Add("g = vg.VisGraph()");
                if (mapPolygons[x].Count != 0)
                {
                    result.Add("print \"    Core Count = 6\"");
                    result.Add("g.build(polys, workers=6)");
                }
                else {
                    result.Add("print \"    Core Count = 1\"");
                    result.Add("g.build(polys)");
                }
                result.Add("print \"    Shortest Path\"");
                foreach (List<string> path in mapPaths[x])
                {
                    for (int y = 0; y < path.Count - 1; y++)
                    {
                        if (y == 0)
                        {
                            result.Add("shortest = g.shortest_path(vg.Point" + path[y] + ", vg.Point" + path[y + 1] + ")");
                            continue;
                        }

                        result.Add("shortest += g.shortest_path(vg.Point" + path[y] + ", vg.Point" + path[y + 1] + ")[1:]");
                    }
                    result.Add("s = str(shortest)");
                    result.Add("s = s[1:-1] + \";\"");
                    result.Add("totalS += s");
                    result.Add("s = str(shortest) + \",\"");
                    result.Add("totalDM += s");
                    result.Add("s = \"\"");
                }
                result.Add("totalS = \"" + (x + 1) + ": \" + totalS[:-1] + \"\\n\"");
                result.Add("totalDM = \"" + (x + 1) + ": [\" + totalDM[:-1] + \"]\\n\"");
                result.Add("fileContentSol += totalS");
                result.Add("fileContentDM += totalDM");
                result.Add("totalS = \"\"");
                result.Add("totalDM = \"\"");
            }
            result.Add("fileContent = \"Solution Output\\n\" + fileContentSol + \"\\n\\nDrawMap.py Output\\n\" + fileContentDM");
            result.Add("f.write(fileContent)");
            result.Add("f.close");
            File.WriteAllLines(Environment.CurrentDirectory + "/aio.py", result);
        }
    }
}
