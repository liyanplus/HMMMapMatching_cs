using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;

namespace HMMMapMatching
{
    class Program
    {
        static void Main(string[] args)
        {            
            var pbfFilePath = args[0];
            var trackingPointFilePath = args[1];
            if (File.Exists(trackingPointFilePath))
            {
                var worker = new YMapMatcher(pbfFilePath, trackingPointFilePath);
            }
            else if (Directory.Exists(trackingPointFilePath))
            {
                foreach (var fileName in Directory.EnumerateFiles(trackingPointFilePath, "*.csv"))
                {
                    if (fileName.IndexOf('.') + 4 == fileName.Length)
                    {
                        try
                        {
                            var worker = new YMapMatcher(pbfFilePath, fileName);
                        }
                        catch (Exception)
                        {
                            continue;
                        }                        
                    }                    
                }
            }
        }
    }
}
