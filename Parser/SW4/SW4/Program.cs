using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;

namespace SW4
{
    class MainClass
    {
        public class CO
        {
            int x;
            int y;
        }

        public class Polygons
        {
            List<CO> polygon;
        }

        public class Map
        {
            List<CO> robots;
            Polygons polygons;
        }

        public void Read()
        {
            var reader = new StreamReader(Environment.CurrentDirectory + "/position.txt");
            string positions = reader.ReadToEnd();
            Console.WriteLine(Environment.CurrentDirectory + "position.txt");
            List<string> lines = positions.Split('\n').ToList();

            List<string> mapRobots = new List<string>();
            List<List<string>> mapPolygons = new List<List<string>>();

            foreach (string line in lines)
            {
                Map newMap = new Map();
                Boolean hashCheck = false;
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
                polygons.Add(polygon);
                mapPolygons.Add(polygons);
            }

            for (int x = 0; x < lines.Count; x++)
            {
                string content = "";
                content += "Robots\n";
                content += mapRobots[x] + "\n\n";
                content += "Polygons\n";
                content += "[";
                foreach (string polygon in mapPolygons[x])
                {
                    content += "[";
                    content += polygon;
                    content += "],";
                }
                content = content.Remove(content.Length - 2);
                content += "]";
                File.WriteAllText(Environment.CurrentDirectory + "/" + x + ".txt", content);
            }


            Console.WriteLine(lines[0]);


        }


        public static void Main(string[] args)
        {
            MainClass test = new MainClass();
            test.Read();
        }
    }
}
