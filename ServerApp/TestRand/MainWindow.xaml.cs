using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
//using NodeInfo;
using ns.graph;
using ns.graph.user_gui;
using ns.wrsn;
//using NJNP;
//using mTs;
//using Nayab;
using Newtonsoft.Json;
using System.IO;
using Path = System.Windows.Shapes.Path;
using Microsoft.Win32;
using System.Timers;
namespace TestRand
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        ns_pen pen = new ns_pen(Brushes.Red, 2);
        //private Wind_net_Stat net_stats_wind;
        int i = 0;
        private int sensor_count = 0;
        private string File_name = string.Empty;
        private int mc_count = 0;
        private int bs_count = 0;
        private fx_graph gf = new fx_graph(400, 400, 0, 0);
        private enum Node_type
        {
            Sensor = 0,
            MC = 1,
            BS = 2
        };
        private Node_type NodeType = Node_type.Sensor;
        private string prev_clicked_node = "";
        int j = 0;
        ns_point last = new ns_point();
        private bool BoolUsePen = false;
        Dictionary<string, ns_node> NodesList = new Dictionary<string, ns_node>();
        Dictionary<string, mobile_charger> MobileChargers = new Dictionary<string, mobile_charger>();
        private string app_log_file = "app_log.txt";
       // private double connect_max_dist = 500;
        private int max_nodes = 50;
        private Timer simTimer;
        private int samples_interval_ms = 1;
        private int simulation_time_ms;
        private double sim_interval;
        private DataLogger dead_nodes_report;
        private DataLogger ns_nodes_charge_stats;
        // this value is used to count the number of intervals;
        private int sim_int_count = 0;
        private string charge_stats_net_report;
        private string charge_q_len_fn;
        private string successful_charged_nodes_fn;
        private int max_screen_buffer_len = 4500;
        //private int k_cluster = 10;
        public MainWindow()
        {
            InitializeComponent();
            //init_sim_timer();
            add_algorithm("dijkstra");
            add_algorithm("A Star");
            add_algorithm("Bread-first search");
            add_algorithm("greedy_best_first_search");
            add_algorithm("hueristic_best_first_search");
            add_algorithm("MC_FIND");
            add_algorithm("Clustering");
            add_algorithm("Dijkstra_Connect");
            //add_algorithm("assign_threshold");
            add_algorithm("send packet");
            this.NodeType = Node_type.Sensor;
            File.WriteAllText(app_log_file, "");//clear



            int layers = net_planning.getOptimumNumberofLayers(2, 1);
            
            int comrange = (int) AppConfig.getComRange();
            int numberofNodes = AppConfig.getNumberOfNodes();
            printf(" optimum layers :{0}", layers);
            printf(" com_range  :{0}", comrange);

            double shifty = Math.Sqrt(3) * numberofNodes / 3;
            create_hex_net(new ns_point(-shifty, 30), layers, comrange);

            //DataLoader("Eng_S_1.dat", true);
           // double mean = DataLoader.GetMean("Eng_S_1.dat", true);
           // printf("mean:{0}", mean);
            this.simulation_time_ms = AppConfig.getSimTimeInMiliSeconds();
            this.samples_interval_ms = AppConfig.get_sample_fx() * 1000;
            this.sim_interval = this.simulation_time_ms / (this.samples_interval_ms);
            printf("sim_interval:{0}\n", this.sim_interval);
            string dir_name = string.Format("./sim_results_{0}", DateTime.Now.ToString("dd_hhmmss"));
            Directory.CreateDirectory(dir_name);
            string name = string.Format("{0}/reports.csv",dir_name);
            this.charge_stats_net_report = string.Format("{0}/charge_reports.csv", dir_name);
            
            
            this.dead_nodes_report = new DataLogger(name);
            name = string.Format("{0}/charge_service_stats.dat", dir_name);
            this.ns_nodes_charge_stats = new DataLogger(name);
            this.charge_q_len_fn = string.Format("{0}/charge_q_len.dat",dir_name);
            this.successful_charged_nodes_fn = string.Format("{0}/sucess_rate.dat", dir_name);

            //ConfigNodes();
            //GenerateThreshold();
            //guassian_window(25, 0.3, 1);
            eng_threshold_method method = AppConfig.getThresholdMethod();
            printf("Threshold method:{0}", method.ToString());
            string table = "MC_ID\tS_ID\t ART\tResponse time \tWQT\tCD\tQL\tCR\n";
            this.ns_nodes_charge_stats.Write(table);
            GenerateThreshold();// _loaction_based();
            double[] win = create_hex_threshold(layers);
            config_optimization_constant_for_MCS();
            //GenerateThresholdByWindow(0, 0.25, win);
            calc_arrival_time();
            alpha_overTmax();
            //clusteringThresholdValue(3);
            derivativeOfGuassian();
            //get_average_inter_arrival_time();
            ns_erf();
            OutputTcpPortFilters();// for wireshark filters.
        }
        private void ns_erf()
        {
            string fn = "erf.dat";
            string output = "";
            double E_max = AppConfig.getRemainingEnergy();
            double E_c = AppConfig.getEngConsumption();
            double alpha = AppConfig.getAlpha();
            double N = AppConfig.getNumberOfNodes();
            double pi_1 = (E_max - 5) / (E_c);
            double pi_2 = 35 / E_c;
            int i = 0;
            double y = 0;
            for (i = 0; i < N; i++)
            {
                double x = i / (N * alpha);
                double p1 = pi_1;// *(i - j);
                double p2 = pi_2 * (ns_window_functions.erf(x) - ns_window_functions.erf(y));
               
                y = x;
                output += string.Format("{0}\t{1}\n", i, p2);
            }

                File.WriteAllText(fn, "");
            File.WriteAllText(fn, output);
        }
        private double get_average_inter_arrival_time()
        {
            //string fn = "der.dat";
            double sum = 0;
            double E_max = AppConfig.getRemainingEnergy();
            double E_c = AppConfig.getEngConsumption();
            double alpha = AppConfig.getAlpha();
            double N = AppConfig.getNumberOfNodes();
            double pi_1 = (E_max - 5) / (E_c);
            double pi_2 = 35 / E_c;
            int i = 0;
           // string output = "";
            for (i = 0; i < N; i++)
            {
                double dx = pi_2 * i * (1 / (alpha * N)) * (1 / (alpha * N));
                double beta_i = (i) / (alpha * N);
                double exp = beta_i * beta_i;
                double w_n = dx * Math.Exp(-0.5 * exp);
                sum += w_n;
            }
            double res = sum / (N);///2;
                                   ///
            printf("the Average inter-constant:{0}\n", res);
            return res;
        }
        private void derivativeOfGuassian()
        {
            string fn = "der.dat";
            double E_max = AppConfig.getRemainingEnergy();
            double E_c = AppConfig.getEngConsumption();
            double alpha = AppConfig.getAlpha();
            double N = AppConfig.getNumberOfNodes();
            double pi_1 = (E_max - 5) / (E_c);
            double pi_2 = 35 / E_c;
            int i = 0;
            string output = "";
            for (i = 0; i < N; i++)
            {
                double dx = pi_2 * i * (1 / (alpha * N)) * (1 / (alpha * N));
                double beta_i = (i) / (alpha * N);
                double exp = beta_i * beta_i;
                double w_n = dx * Math.Exp(-0.5 * exp);
                output += string.Format("{0}\t{1}\n", i, w_n);
            }
            File.WriteAllText(fn, "");
            File.WriteAllText(fn, output);
        }
        private void alpha_overTmax()
        {
            double E_max = AppConfig.getRemainingEnergy();
            double E_c = AppConfig.getEngConsumption();
            double alpha = AppConfig.getAlpha();
            double N = AppConfig.getNumberOfNodes();
            double pi_1 = (E_max - 5) / (E_c);
            double pi_2 = 35 / E_c;
            int i = 0;
            //double beta_i = 0;
            string win = "";
            double t_max = 20;//min
            for (i = 1; i < 60; i++)
            {
                t_max = 30 * i;
                double a = 2 *  Math.Log(pi_2 / Math.Abs(pi_1 - t_max));
                a = Math.Sqrt(a);
                a = N * alpha * a;
                win += string.Format("{0}\t{1}\n",i, a);

            }
            File.WriteAllText("arx.dat", "");
            File.WriteAllText("arx.dat", win);
        }
         private void calc_arrival_time()
        {
            double E_max = AppConfig.getRemainingEnergy();
            double E_c = AppConfig.getEngConsumption();
            double alpha = AppConfig.getAlpha();
            double N = AppConfig.getNumberOfNodes();
            double C1 = (E_max - 5)/(E_c);
            double C2 = 35 / E_c;
            int i = 0;
            double beta_i = 0;
            string win = "";
            for (i = 0; i < N; i++)
            {
                beta_i = (i) / (alpha * N);
                double exp = Math.Pow(beta_i, 1);
                double w_n = C1 - (C2 * Math.Exp(-0.5 * exp));
                win += string.Format("{0}\n", w_n);
            }
            File.WriteAllText("ar.dat", "");
            File.WriteAllText("ar.dat", win);
        }
        private void GenerateThresholdByWindow(double shift, double scale, double[] win)
        {
            int i = 0;
            string fn = "win.dat";
            string win_text = "";
            Random r = new Random(DateTime.Now.Millisecond);
            foreach(var fx in this.NodesList)
            {
                var node = fx.Value;
                double s = r.Next(1, 30);
                if(node.NodeType == Node_TypeEnum.Sensor_node)
                {
                    node.PowerConfig.energy_threshold = s + (scale * win[i]);


                    win_text += string.Format("{0}\n", s + (scale * win[i]));
                    i++;
                }
            }
            File.WriteAllText(fn, "");
            File.WriteAllText(fn, win_text);
        }
        private void clusteringThresholdValue(int k)
        {
            int N = (int)Math.Ceiling((double)(this.sensor_count / k));
            double alpha = AppConfig.getAlpha();
            double s = 35;
            double shift = 5;
            double[] g1 = guassian_window3(N, alpha, s, shift);
            int i = 0;
            string id;
            string outfx = "";
            for (int j = 0; j < k; j++)
            {
                for (i = 0; i < N; i++)
                {
                    id = string.Format("S_{0}", (N * j) + i + 1);
                    double t = g1[i];
                    this.NodesList[id].PowerConfig.energy_threshold = t;
                    outfx += string.Format("{0}\t{1}\n", id, t);
                }
                s = 35 - (k * (j + 1));
                shift = 5 + k;

                g1 = guassian_window3(N, alpha, s, shift);
            }
            File.WriteAllText("windx.dat", "");
            File.WriteAllText("windx.dat", outfx);
        }
        private void mc_OnNodeChargeDoneStats(mobile_charger mc, ns_node_stats service_stats)
        {
           
            string fmt = "{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}\n";
            
            this.ns_nodes_charge_stats.AppendFormat(fmt,
                 mc.tag,
                service_stats.node_id,
                service_stats.getArrivalTime(),
                service_stats.getResponseTime(),
                service_stats.getWaitingInQueueTime(),
                service_stats.getChargingDuration(),
                
                service_stats.queue_len_at,
                service_stats.getChargeRatio()
               
               );
               
        }
        private void assign_threshold()
        {
            try
            {
                this.screen.Text = "";
                string bs_id = AppConfig.getConfigValue("base_station_id", "BS_1");
                if (!this.NodesList.ContainsKey(bs_id))
                {
                    printf("NO BS_1");
                    return;
                }

                ns_node bs = this.NodesList[bs_id];
                net_packet assign_packet = new net_packet(bs_id, "", packetType.assign_threshold);
                assign_packet.SetNumberofNodes(this.sensor_count);
                bs.SendEngThresholdPacketbyFlooding(assign_packet);
                printf("Assign Threshold DONE>---");
            }catch(Exception ex)
            {
                printf("Exception in assign_threshold:{0}", ex.Message);
            }
           
        }
        private double GetConfigValue(string key , double def)
        {
            string value = AppConfig.getConfigValue(key);
            if (string.IsNullOrEmpty(value)) return def;
            printf("{0}:{1}", key, value);
            return double.Parse(value);
        }
        public void config_optimization_constant_for_MCS()
        {
            double optim_const = get_average_inter_arrival_time();
            double alpha = AppConfig.getAlpha();
            foreach(var mc in this.MobileChargers)
            {
               // mc.Value.
                mc.Value.optimization_constant = optim_const / alpha;
                mc.Value.enable_threshold_switch = true;
            }
        }
        public void ConfigNodes()
        {
            //double[] thrs = guassian_window(this.NodesList.Count, 0.444, 0.5);
            //int i = 0;
            double the = AppConfig.getEngConsumption();
            double pk = AppConfig.getEnTxRx();
            double cr = AppConfig.getChargeRate();
            double max_eng = AppConfig.get_sensor_node_battery_cap();
            double mc_max_eng = AppConfig.get_mobile_charger_battery_cap();
            double sleep_mode_eng = AppConfig.getSleepModeEng();
            bool sleep_mode_enable = AppConfig.getSleepModeStatus();
            eng_threshold_method method = AppConfig.getThresholdMethod();
            Random r = new Random(DateTime.Now.Millisecond);
            double eng_thres_constant = AppConfig.get_eng_threshold_constant();
            int int_start = AppConfig.net_timer_time_range_start();
            int int_end = AppConfig.net_timer_time_range_end();
            double r_en = AppConfig.getRemainingEnergy();
            foreach(var node in this.NodesList)
            {
                if (node.Value.NodeType == Node_TypeEnum.Sensor_node)
                {
                    node.Value.PowerConfig.BatteryCapacity = r_en; // remaining energy
                    node.Value.PowerConfig.initial_energy = max_eng;
                    node.Value.PowerConfig.energy_consumptionRate = the;
                    node.Value.PowerConfig.PacketEnergy = pk;
                    node.Value.PowerConfig.ChargeRate = cr;
                    node.Value.PowerConfig.EnableSleepMode(sleep_mode_enable);
                    node.Value.PowerConfig.Sleep_mode_En = sleep_mode_eng;
                    node.Value.SetNetTimerTimeRange(int_start, int_end);
                    if(method == eng_threshold_method.constant)
                    {
                        node.Value.PowerConfig.energy_threshold = eng_thres_constant;
                        continue;
                    }
                    if(method == eng_threshold_method.random)
                    {
                        node.Value.PowerConfig.energy_threshold = r.Next(20, 40);
                    }
                }
                if(node.Value.NodeType == Node_TypeEnum.Mobile_Charger)
                {
                    node.Value.PowerConfig.ChargeRate = cr;
                    node.Value.PowerConfig.BatteryCapacity = mc_max_eng;
                    node.Value.PowerConfig.initial_energy = mc_max_eng;
                    node.Value.PowerConfig.EnableSleepMode(sleep_mode_enable);
                    node.Value.PowerConfig.Sleep_mode_En = sleep_mode_eng;
                    node.Value.SetNetTimerTimeRange(int_start, int_end);
                }
               // i++;
            }
            if(method== eng_threshold_method.guassian)
            {
              //GenerateThreshold();
                //clusteringThresholdValue(this.k_cluster);
               // GenerateThresholdByWindow(0, 10, ns_window_functions.hanning_window(150, 1, 0.50, 1));
            }
        }
        
        private void threshold_based_color(ns_node node, string fn)
        {
             double alpha = AppConfig.getAlpha();
             double thres = node.location.ThresholdFromLocation(0.01, 0.024, 0.01) ;//% 40;
            string txt = string.Format("{0}\t{1}\n",node.tag, thres);
            node.PowerConfig.energy_threshold = thres;
            File.AppendAllText(fn, txt);
        }

        public void GenerateThreshold_loaction_based()
        {
            double alpha = AppConfig.getAlpha();
            int N = AppConfig.getNumberOfNodes();
            double[] thrs = guassian_window4(N, alpha, 35);
            int i = 0;
            string fn= "loc_thres.dat";
            File.WriteAllText(fn, "");
            //int N = this.NodesList.Count;
            foreach (var item in this.NodesList)
            {
                if (item.Value.NodeType == Node_TypeEnum.Sensor_node)
                {

                  //  item.Value.PowerConfig.energy_threshold = thrs[i] + 5;
                   // item.Value.PowerConfig.threshold_switch = (thrs[i] + 5) / (2);
                    threshold_based_color(item.Value, fn);
                    i++;
                }

            }
        }
        public double[] y_loc_node()
        {
            int N = AppConfig.getNumberOfNodes();
            double[] a = new double[N];
            int i = 0;
             foreach (var item in this.NodesList)
             {
                 if (item.Value.NodeType == Node_TypeEnum.Sensor_node)
                 {
                     a[i] = 5 + (item.Value.location.y % 255) % (40);
                     i++;
                 }
             }
             return a;
        }
        public void GenerateThreshold_discrete()
        {
            double alpha = AppConfig.getAlpha();
            int N = AppConfig.getNumberOfNodes();
            double[] thrs = guassian_window4(N, alpha, 35);
            int i = 0;
            //int N = this.NodesList.Count;
           
            double[] fxs = { 40, 35, 30, 25, 20, 15, 10};
            int max = fxs.Length;
            foreach (var item in this.NodesList)
            {
                if (item.Value.NodeType == Node_TypeEnum.Sensor_node)
                {

                    double y = (item.Value.location.y % 255) % (30);
                    double z = y + (i % 10) * (1.2);
                    double th = y + z + 2;
                    item.Value.PowerConfig.energy_threshold = th;// +8;// 40 - (1.2 * (i % max));
                    item.Value.PowerConfig.threshold_switch = (th + 5) / (2);
                    i++;
                }

            }
        }
        public void GenerateThreshold()
        {
            double alpha = AppConfig.getAlpha();
            int N = AppConfig.getNumberOfNodes();
            double[] thrs = guassian_window4(N, alpha, 35);
            int i = 0;
            //int N = this.NodesList.Count;
            foreach(var item in this.NodesList)
            {
                if (item.Value.NodeType == Node_TypeEnum.Sensor_node)
                {

                    item.Value.PowerConfig.energy_threshold =  thrs[i] + 5;
                    item.Value.PowerConfig.threshold_switch = (thrs[i] + 5) / (2);
                    i++;
                }
                
            }
        }

        public double[] guassian_window3(int N, double alpha, double amp, double shift)
        {
            double[] w = new double[N];
            int i = 0;
            // ((3*x*(x+1))+1)*(40*0.5*sqrt(3))
            double N_2 = N / 2.0;
            //string fn = "gus3.dat";
            //File.WriteAllText(fn, "");
           // string outf = "";
            //double[] w1 = new double[N / 2];
            double sum = 0.0;
            for (i = 0; i < N; i++)
            {
                double exp = ((i) / (alpha * N));
                double res = shift + (amp * Math.Exp(-0.5 * exp * exp));
                sum += res;
                w[i] = res;
               // outf += string.Format("{0}\n", res);
            }
           
           // printf("Guassian Mean:{0}", sum / N);
            
            //File.WriteAllText(fn, outf);
            return w;
        }
        public double[] guassian_window4(int N, double alpha, double amp)
        {
            double[] w = new double[N];
            int i = 0;
            // ((3*x*(x+1))+1)*(40*0.5*sqrt(3))
            double N_2 = N / 2.0;
            string fn = "gus.dat";
            File.WriteAllText(fn, "");
            string outf = "";
            //double[] w1 = new double[N / 2];
            double mc = AppConfig.getNumberofChargers();

            int m = (int)Math.Ceiling(N / (1.0));
           // double en_c = 
            double sum = 0.0;
            for (i = 0; i < N; i++)
            {
                double fx = ( i)% (m);
                double exp = (Math.Pow((fx) / (m * alpha), 2));
                double res = amp * Math.Exp(-0.5 * exp);
                sum += res;
                w[i] = res;
            }
            //double[] w2 = w1.Reverse().ToArray();
            // w2.CopyTo(w,0);
            // w1.CopyTo(w, N / 2);
            printf("Guassian Mean:{0}", sum / N);
            foreach (var x in w)
            {
                outf += string.Format("{0}\n", x);
            }
            File.WriteAllText(fn, outf);
            return w;
        }
        public double[] guassian_window2(int N, double alpha, double amp)
        {
            double[] w = new double[N];
            int i = 0;
            // ((3*x*(x+1))+1)*(40*0.5*sqrt(3))
            double N_2 = N / 2.0;
            string fn = "gus.dat";
            File.WriteAllText(fn, "");
            string outf = "";
            //double[] w1 = new double[N / 2];
            double sum = 0.0;
            for (i = 0; i < N; i++)
            {
                double exp = ((i) / (alpha * N));
                double res = amp * Math.Exp(-0.5 * exp * exp);
                sum += res;
                w[i] = res;
            }
            //double[] w2 = w1.Reverse().ToArray();
            // w2.CopyTo(w,0);
            // w1.CopyTo(w, N / 2);
            printf("Guassian Mean:{0}", sum / N);
            foreach (var x in w)
            {
                outf += string.Format("{0}\n", x);
            }
            File.WriteAllText(fn, outf);
            return w;
        }
        public double[] guassian_window(int N, double alpha, double amp)
        {
            double[] w = new double[N];
            int i = 0;
            // ((3*x*(x+1))+1)*(40*0.5*sqrt(3))
            double N_2 = N / 2.0;
            string fn = "gus.dat";
            File.WriteAllText(fn, "");
            string outf="";
            double[] w1 = new double[N / 2];
            double sum = 0.0;
            for (i = 0; i < N; i++) 
            {
                double exp = ((i - N_2) / (alpha * N_2));
                double res = amp * Math.Exp(-0.5 * exp * exp);
                sum += res;
                w[i] = res;
            }
            //double[] w2 = w1.Reverse().ToArray();
           // w2.CopyTo(w,0);
           // w1.CopyTo(w, N / 2);
            printf("Guassian Mean:{0}", sum / N);
           foreach(var x in w)
           {
               outf += string.Format("{0}\n", x);
           }
                File.WriteAllText(fn, outf);
            return w;
        }
        private void init_sim_timer()
        {
            if(this.simTimer!=null)
            {
                printf("Timer is running .. stopping timer");
                Stop_Simulation();

            }

            //double sim_time = AppConfig.getSimTimeInMiliSeconds();
            //printf("sim_time:{0}\n", sim_time);
           // double interval = this.simulation_time_ms / (this.samples_interval_ms);
            this.simTimer = new Timer(this.samples_interval_ms);
            this.simTimer.Elapsed += simTimer_Elapsed;
            this.simTimer.Start();
            printf("Simulation Timer Started successfully\n");
        }
        private void Stop_Simulation()
        {
            if (this.NodesList.Count == 0) return;
            this.is_sim_start = false;
            foreach (var fx in this.NodesList)
            {
                ns_node node = fx.Value;

                node.Stop();
            }
            this.simTimer.Stop();
            ExportPower_Click(null, null);
            MessageBox.Show("Simulation Stopped");
           
        }
        private bool is_first_row_report = false;
        private string first_charge_request_fn = "first_charge_req.dat";
        private void ExportFirstChargeRequestTime()
        {
            File.WriteAllText(this.first_charge_request_fn,"");
            string str = "";
            double[] reqTimes = new double[this.sensor_count];
            int j = 0;
            foreach (var fx in this.NodesList)
            {
                var node = fx.Value;
                if(node.NodeType == Node_TypeEnum.Sensor_node)
                {
                    double fxx = node.FirstChargeRequestTime(false);
                    if (fxx != 0)
                        str += string.Format("{0}\t{1}\n", node.tag, fxx);
                }
            }
          //  reqTimes =
            File.WriteAllText(this.first_charge_request_fn, str);
        }
        private void ExportReports()
        {
            int count = 0;
            int node_count = this.NodesList.Count;
            double mean = 0.0;
            double sum = 0.0;
            double thrpt = 0.0;
            string cols = "";
            double[] pow = new double[node_count+5];
            double charge_requests = 0.0;
            int k = 0;
            //string sep = ",";
            foreach(var kvp in this.NodesList)
            {
                ns_node node = kvp.Value;
                double power = node.PowerConfig.BatteryCapacity;
                double threshold = node.PowerConfig.energy_threshold;
                pow[k] = power;


                //sep = k == node_count - 1 ? "" : ",";
                cols += string.Format("{0}_eng,", node.tag);
                 k++;
                if(node.NodeType == Node_TypeEnum.Sensor_node)
                {
                    sum += power;
                    thrpt += (node.getRxPackets() + node.getTxPackets()) / 2;
                    charge_requests += node.PowerConfig.charge_request_packets_tx;
                    if (power <= 1) 
                    {
                        count++;
                    }
                }
            }
            mean = sum / (this.sensor_count);
            thrpt = thrpt / (this.sensor_count);
            pow[k] = mean;
            pow[k + 1] = thrpt;
            pow[k + 2] = count;
            pow[k + 3] = (this.sensor_count - count) * 100 / (this.sensor_count);
            pow[k + 4] = (charge_requests) / (this.sensor_count);
            if (!this.is_first_row_report)
            {
                cols = cols + "average_power,throughtput,dead_nodes,survival_rate,charge_req\n";
                this.dead_nodes_report.Write(cols);
                this.is_first_row_report = true;
            }
            
           // count = 
            this.dead_nodes_report.WriteLineMultiCsv(pow);
            this.ExportChargeStatistics();
            this.ExportFirstChargeRequestTime();
        }
        
        
        private void GetChargingQlenReport()
        {
            double sum = 0;
            string fx = "";
            string success = "";
            double sum2 = 0;
            //int k = 0;
            //int count = this.MobileChargers.Count;
            foreach(var mc in this.MobileChargers)
            {

                fx += string.Format("{0}\t", mc.Value.GetQueueLen());
                sum += mc.Value.GetQueueLen();

                double x=mc.Value.getNumberOfChargedNodes();

                success += string.Format("{0}\t", x);
                sum2 += x;
            }
            fx += string.Format("{0}\t{1}\n", sum, (this.sensor_count - sum));
            success += string.Format("{0}\n", sum2);
            File.AppendAllText(this.charge_q_len_fn, fx);
            File.AppendAllText(this.successful_charged_nodes_fn, success);
        }
        private void ExportChargeStatistics()
        {
            
            GetChargingQlenReport();
        }
       private void simTimer_Elapsed(object sender, ElapsedEventArgs e)
        {

            if (this.sim_int_count >= this.sim_interval)
            {
                Stop_Simulation();
                return;
            }

            this.sim_int_count++;
            ExportReports();
            printf("Sim interval Round:{0}\n", this.sim_int_count);
          
        }
        private void Clustering(Dictionary<string,ns_node> parentof)
       {
           //parentof.Reverse();
          // List<ns_node> cluster_nodes = new List<ns_node>();
           int i = 0;
         // var dict = parentof.Reverse();
           Stack<string> stak = new Stack<string>();
           Dictionary<string, ns_node> new_nodes = new Dictionary<string, ns_node>();
            foreach(var kvp in parentof)
            {
                stak.Push(kvp.Key);
            }
            while(stak.Count!=0)
            {
                var item = stak.Pop();
                var current = parentof[item];
                string c_tag = current.tag;
                if (!parentof.ContainsKey(c_tag))
                {
                    printf("node :{0}\n", c_tag);
                   // new_nodes[c_tag] = current;
                   // new_nodes.Add(c_tag, current);
                    i++;
                }
            }
           
            
            
            printf("Number of clusters :{0}", i);
           if(i>1)
            ns_dij(new_nodes);
       }
        private  List<ns_node> ConstructPath(Dictionary<string, ns_node> parentof, ns_node start, ns_node current)
        {
            List<ns_node> path = new List<ns_node>();

            while (current.tag != start.tag)
            {
                path.Add(current);
                current = parentof[current.tag];
                printf("{0},", current.tag);
            }
            path.Add(start);
            path.Reverse();
            //int i = 0;
           // Draw(path);


            return path;
        }
        private bool containsNode(string id)
        {
            return this.NodesList.ContainsKey(id);
        }
        public void hexConnect()
        {
            int count = this.NodesList.Count;
            int j = 0;
            //int mc =0;
            string current ="";
            string next ="";
            string left ="";
            for (j = 0; j < count / 2; j++)
            {
                current = string.Format("S_{0}", j + 1);
                next = string.Format("S_{0}", (j + 7) % (count));
                left = string.Format("S_{0}", (j + 1 + 1) % (count));
                if (this.containsNode(current) && this.containsNode(next))
                {
                    var c_node = this.NodesList[current];
                    var next_node = this.NodesList[next];
                    double dist = c_node.mdistance(next_node);
                    ns_link.RenderLink(c_node, next_node, dist, this.mcanvas.Children);
                    c_node.addLink(next_node, dist);
                    if(this.containsNode(left))
                    {
                        var left_node = this.NodesList[left];
                        dist = c_node.mdistance(left_node);
                        ns_link.RenderLink(c_node, left_node, dist, this.mcanvas.Children);
                        c_node.addLink(left_node, dist);
                    }

                }

            }
        }
        public void ns_dij(Dictionary<string,ns_node> nodes)
        {
            //Dictionary<string, List<ns_node>> dict = new Dictionary<string, List<ns_node>>();
            Dictionary<string, double> links = new Dictionary<string, double>();
            Dictionary<string, ns_node> parentof = new Dictionary<string, ns_node>();
            string t1 = "";
            string t2 = "";
            double dist = 0;
            double min = 0;
            string min_tag = "";
           // int i = 0;
            int j = 0;
           // double mean = 0.0;
            Dictionary<string, bool> marked = new Dictionary<string, bool>();
            List<ns_node> neigbors = new List<ns_node>();
            string link1 = "";
            string link2 = "";
            
            foreach(var g1 in nodes)
            {
                ns_node n1 = g1.Value;
                t1 = g1.Key;
                neigbors.Clear();
                foreach(var g2 in nodes)
                {
                    t2 = g2.Key;
                    if (t1 == t2) continue;
                    link1 = string.Format("{0}-{1}", t1, t2);
                    link2 = string.Format("{0}-{1}", t2, t1);

                    if (marked.ContainsKey(link1) && marked.ContainsKey(link2)) continue;

                    ns_node n2 = g2.Value;
                    if(j==0)
                    {
                        min = n1.edistance(n2);
                        min_tag = t2;
                    }
                    dist = n1.edistance(n2);
                    min = min < dist ? min : dist;
                    min_tag = min == dist ? t2 : min_tag;
                    if(min==dist)
                    {
                       // neigbors.Add(n2);

                    }

                    j++;
                }
                j = 0;
                /*foreach(ns_node n in neigbors)
                {
                    link1 = string.Format("{0}-{1}", t1, n.tag);
                    link2 = string.Format("{0}-{1}", n.tag, t1);
                    marked[link1] = true;
                    marked[link2] = true;
                    n1.addLink(n);

                    ns_link.RenderLink(n1, n, min, mcanvas.Children);
                }*/
                link1 = string.Format("{0}-{1}", t1, min_tag);
                link2 = string.Format("{0}-{1}", min_tag, t1);
               
                marked[link1] = true;
                marked[link2] = true;
                j= 0;
                ns_node target = nodes[min_tag];
                ns_link.RenderLink(n1, target, min, mcanvas.Children);

                n1.addLink(target, min);
                if (!parentof.ContainsKey(min_tag))
                {
                    //links[link1] = min;
                    // mean += min;
                    // printf("{0}:{1:0}\n", link1, min);
                    parentof[min_tag] = n1;
                }
                
            }
            //printf("Mean:{0}\n", mean / nodes.Count);
            Clustering(parentof);

        }
        public ns_node getBS()
        {
            string id = AppConfig.getConfigValue("base_station_id", "BS_1");
            if(!this.NodesList.ContainsKey(id))
            {
                return this.NodesList.First().Value;
            }
            return this.NodesList[id];
        }
        public  void ns_Dijkstra_connect(Dictionary<string, ns_node> nodes, double max_dist)
        {

            PriorityQueue<double, ns_node> frontier = new PriorityQueue<double, ns_node>((d1, d2) =>
            {
                return d1.priority <= d2.priority;
            });
            ns_node src = getBS();
            frontier.Enqueue(src, 0);
            Dictionary<string, double> cost_so_far = new Dictionary<string, double>();
            cost_so_far[src.tag] = 0;
            Dictionary<string, ns_node> parentof = new Dictionary<string, ns_node>();
            List<ns_node> path = new List<ns_node>();
            ns_node dst = src;
            int jk = 0;
            while (!frontier.IsEmpty())
            {
                var cur_item =  frontier.First();
                jk++;
                ns_node current = cur_item.Value;

                foreach (var kvp in nodes)
                {
                    ns_node next = kvp.Value;
                    if (next.tag == current.tag) continue;
                    double cost = current.edistance(next);
                    if (cost <= max_dist && !cost_so_far.ContainsKey(next.tag))
                    {
                        cost_so_far[next.tag] = cost;
                        double priority = cost;
                        frontier.insert(next, priority);

                        parentof[next.tag] = current;

                        ns_link.RenderLink(current, next, cost, mcanvas.Children);
                        current.addLink(next, cost);
                    }
                }
            }

        }
        public void Mc_Connect()
        {
            foreach(var mc in this.MobileChargers)
            {

            }
        }
        public void ConnectAll()
        {
            double max_len = AppConfig.getMaxLinkLen();
            printf("max len:{0}", max_len);
            ns_Dijkstra_connect(this.NodesList, max_len);
            //ns_dij(this.NodesList);
           // ns_dij(this.MobileChargers);
        }
        public void Find_NeartestNodesTo(ns_node src)
        {
            ns_point src_loc = src.location;
            double max_dist = 200;
            double dist =   0.0;
            foreach(var kvp in this.NodesList)
            {
                ns_node node = kvp.Value;
                if(node.tag == src.tag) continue;
                dist = src.edistance(node);
                Random rn = new Random(DateTime.Now.Millisecond);
                if (dist <= max_dist) 
                {
                    printf("{0}--{1}:{2}\n", src.tag, node.tag, dist);
                    src.addLink(node, dist);
                    ns_link link = new ns_link(src, node);
                  
                   // link.cost = rn.Next(40);
                    link.RenderTo(mcanvas.Children);
                }
            }
        }
       public void create_new_net()
        {
            randomNet net = new randomNet(max_nodes, 883, 883);
            net.GenerateRandomNet();

            foreach (var kvp in net.Nodes)
            {
                //this.
                ns_node sn = kvp.Value;
                string id = kvp.Key;
                sn.OnPowerDown += sn_OnPowerDown;
                add_to_nodelist(id, sn);

                this.sensor_count++;
                sn.SetNetId(getnet_id());
            }
        }
        private void paper_algorithm_hex(ns_point p1, int psi, double tr)
        {
            int i, j;
            double size = tr;
            for (i = -psi; i <= psi; i++)
            {
                for (j = -psi; j <= psi; j++)
                {
                    int r = -(i + j);
                    int q = i;
                    if(abs(i+j)<=psi)
                    {
                        //var x = size * (sqrt(3) * q + (sqrt(3) / 2) * r);
                        //var y = size * (r * 3.0 / 2);
                        double x = p1.x+ tr * ( sqrt(3)*q + (sqrt(3) / 2) * r);
                        double y = p1.y+ tr * (r * (3.0 / 2));
                        ns_point p = new ns_point(x, y);
                        string title = string.Format("({0},{1})", r + psi, q + psi);
                        ns_node node = new ns_node(title, p);
                        string tt = string.Format("S_{0}", this.sensor_count);
                        this.sensor_count++;
                        NodesList.Add(tt, node);
                        node.RenderTo(mcanvas.Children);
                        //create_hexagon(p, 6, tr);
                    }
                }
            }
        }
        private void createMutil_cluster_net(ns_point p1)
        {
            string latex = create_clusters_net(p1, 1, 1.75 * 40 * sqrt(3));
            toLatexTikz(latex, "multi_cluster11q.tex");
        }
        private string create_clusters_net(ns_point p1, int layers, double ra)
        {
            int r = layers;
            int i = 0, j = 0;
            string latex = "";
            for (i = -r; i <= r; i++)
            {
                for (j = -r; j <= r; j++)
                {
                    int k = -(i + j);
                    if (abs(k) <= r)
                    {
                        hex hex = new hex(k, i);
                        ns_point p = hex.toPointy(p1, ra);
                       // latex += create_hexagon(p, 6, ra, i == j ? Node_type.MC : Node_type.Sensor);
                        latex+=create_hex_net(p, 1, 40);
                        //printf("x:{0} \t y:{1} , k:{2}", i, j, k);
                    }
                }
            }
            return latex;
        }
        private bool inRange(int x,  int max , int min)
        {
            return (max >= x) && (x >= min);
        }
        private void ns_add_link_hex(string prev , string current , int i , int j)
        {
            printf("{0}:({1},{2}):{3}\n", current, i, j, abs(i + j));
            if(i==j)
            {
                printf("Same Node :{0}\n", current);
            }
            if (true) return;
            if (string.IsNullOrEmpty(prev) || string.IsNullOrEmpty(current)) return;
            if (prev == current) return;
            if (!this.containsNode(prev) && !this.containsNode(current)) return;
            var current_node = this.NodesList[current];
            var prev_node = this.NodesList[prev];
            double dist = prev_node.mdistance(current_node);
            ns_link.RenderLink(prev_node, current_node, dist, mcanvas.Children);
            prev_node.addLink(current_node, dist);
        }
        private void hex_connetc_algorithm(int layers )
        {
            int r = layers;
            int i = 0, j = 0;
            string prev = "";
            string current = "";
            int sn_index = 0;
            int mc_index = 0;
            int bs_index = 0;
            for (i = -r; i <= r; i++)
            {
                for (j = -r; j <= r; j++)
                {
                    int k = -(i + j);
                    if(abs(k)<=r)
                    {
                       
                        if (i == 0 && j == 0)
                        {
                            prev = current;
                            current = string.Format("BS_{0}", bs_index + 1);
                            ns_add_link_hex(prev, current,i,j);
                            bs_index++;
                            continue;
                        }
                        if (j == 0 && inRange(i, 1, -1))
                        {
                            prev = current;
                            current = string.Format("MC_{0}", mc_index + 1);
                            mc_index++;
                            ns_add_link_hex(prev, current,i,j);
                            continue;
                        }
                        if (i == 0 && inRange(j, 1, -1))
                        {
                            prev = current;
                            current = string.Format("MC_{0}", mc_index + 1);
                            ns_add_link_hex(prev, current,i,j);
                            mc_index++;
                            continue;
                        }
                        prev = current;
                        current = string.Format("S_{0}", sn_index + 1);

                        sn_index++;

                        ns_add_link_hex(prev, current , i, j);
                    }
                }
            }
        }
        private double[] create_hex_threshold(int layers)
        {
            int n_nodes = AppConfig.getNumberOfNodes();
            int config_mc_count = AppConfig.getNumberofChargers();
            int r = layers;
            int i = 0, j = 0;
            int m = 0;
            double[] win = new double[n_nodes];
            for (i = -r; i <= r; i++)
            {
                for (j = -r; j <= r; j++)
                {
                    int k = -(i + j);
                    if (abs(k) <= r)
                    {
                       
                        if (i == 0 && j == 0)
                        {
                           
                            continue;
                        }
                        if (k <= 0 && inRange(j, 1, -config_mc_count) && (this.mc_count < config_mc_count))
                        {
                           
                            continue;
                        }
                        if (k >= 0 && inRange(i, config_mc_count, 1) && (this.mc_count < config_mc_count))
                        {
                           
                            continue;
                        }

                        if (m < this.sensor_count) 
                        {
                            double thres = 5 + 5 * ns_window_functions.ham_thres(i, j, n_nodes, k );
                            //double thres = 5 + 5 * ns_window_functions.getGuassianSample(m, 0.3, 15, n_nodes);
                            win[m] = thres;
                            //printf("thres:S{0}:{1}\n", m, thres);

                            m++;
                        }

                    }
                }
            }
            return win;
        }
        private string create_hex_net(ns_point p1,int layers, double ra)
        {
            /// return latex figure.
            /// 
            int n_nodes = AppConfig.getNumberOfNodes();
            int config_mc_count = AppConfig.getNumberofChargers();
           // ra = ra*3 / Math.Sqrt(3);
            int r = layers;
            int i = 0, j = 0;
            string latex = "";
            for (i = -r; i <= r; i++)
            {
                for (j = -r; j <= r; j++)
                {
                    int k = -(i + j);
                    if (abs(k) <= r)
                    {
                        hex hex = new hex(k + r, i + r);
                        double thres = 5 + 5 * ns_window_functions.ham_thres(i, j, 150, i);
                        printf("{0}\n", thres);
                        ns_point p = hex.toPointy(p1, ra);
                        if (i == 0 && j == 0)
                        {
                            latex += create_hexagon(p, 6, ra, Node_type.BS);
                            continue;
                        }
                        if (k<=0 && inRange(j, 1, -config_mc_count) && (this.mc_count < config_mc_count))  
                        {
                            latex += create_hexagon(p, 6, ra,  Node_type.MC);
                            continue;
                        }
                        if (k>=0 && inRange(i, config_mc_count,1) && (this.mc_count < config_mc_count))
                        {
                            latex += create_hexagon(p, 6, ra, Node_type.MC);
                            continue;
                        }
                        /*if(i==0 && inRange(j,0,-1))
                        {
                            latex += create_hexagon(p, 6, ra, Node_type.MC);
                            continue;
                        }*/
                        if(this.sensor_count <n_nodes)
                        latex += create_hexagon(p, 6, ra,Node_type.Sensor);
                       //printf("x:{0} \t y:{1} , k:{2}", i, j, k);
                    }
                }
            }
            return latex;
        }
        private int abs(int x)
        {
            return Math.Abs(x);
        }
        private ns_point  createHexofHex(ns_point center , double r , int n)
        {
            ns_point p = new ns_point();
           // create_hexagon(center, 6, 3*r);
            //create_polygon(center, 8, 3 * r);
            add_MC(center);
            for (int i = 0; i < n; i++) 
            {
                double ang = 60 * i;
                double size = r * Math.Sqrt(3);
                double x = center.x + size * cos(ang);
                double y = center.y + size * sin(ang);
                p = new ns_point(x, y);
                create_hexagon(p, 6, r);
            }
            return p;
        }
        private int graycode(int i)
        {
            return (i >> 1) ^ i;
        }
        private ns_point createNetWithLayers(ns_point center, double r, int layers, int n)
        {
            ns_point p = new ns_point();
            add_MC(center);
            int m = n;
            for (int j = 0; j < layers; j++)
            {
                m = (j + 1) * n;
                for (int i = 0; i < m; i++)
                {
                    double ang = (360 / m) * (i+j);
                    double shift = j > 0 && i % 2 == 0 ? 1.66 : Math.Sqrt(3);
                    double size = (j + 1) * r * shift;
                    double x = center.x + size * cos(ang);
                    double y = center.y + size * sin(ang);
                    p = new ns_point(x, y);
                    create_hexagon(p, 6, r);
                }
            }

            return p;
        }
        private void simple_topo()
        {
            string latex = "";
            ns_point p1 = new ns_point(100, 100);
            latex += honeycomb2(p1, 3, 3, 40);
            ns_point p2 = new ns_point(315, 100);
            latex += honeycomb2(p2, 3, 3, 40);
            ns_point p3 = new ns_point(460, 220);
            latex += honeycomb2(p3, 3, 3, 40);
            toLatexTikz(latex, "fig2.tex");

        }
        private void CreateMyHexNet()
        {
            string latex = honeycomb(7,5,40);
            toLatexTikz(latex, "hexa.tex");
        }
        private double sqrt(double x)
        {
            return Math.Sqrt(x);
        }
        private double cos(double angle_deg)
        {
            double rad = Math.PI * angle_deg / 180;
            return Math.Cos(rad);
        }
        private double sin(double angle_deg)
        {
            double rad = Math.PI * angle_deg / 180;
            return Math.Sin(rad);
        }
        private string hexaToLatex(List<ns_point> points, ns_point center , Node_type type)
        {
            string latex = "";


            int i = 0;
            foreach (var p in points)
            {
                string app = i < points.Count - 1 ? "--" : "";
                latex += string.Format("({0:0.00},{1:0.00}){2} ", (p.x / 50), (p.y / 50), app);
                i++;
            }
            latex = string.Format("\\draw {0};\n", latex);
            string id = string.Format("S_{0}", this.sensor_count);
            if(type==Node_type.BS)
            {
                id = string.Format("BS_{0}", this.bs_count);
            }
            if(type== Node_type.MC)
            {
                id = string.Format("MC_{0}", this.mc_count);
            }
            latex += string.Format("{0}\n", center.toLatexCircle(id, 6, 50, id));
            return latex + "\n";
        }
        private string honeycomb2(ns_point center  , int w , int h, double r)
        {
            string latex = "";
            double x0 = center.x;
            double y0= center.y;
            double hw = r * Math.Sqrt(3);

            for (int i = 0; i < w;i++)
            {
                for (int j = 0; j < h; j++)
                {
                    double x_ij = x0 + hw * (i - j*cos(60));
                    double y_ij = y0 + hw * (j * sin(60));
                    ns_point p = new ns_point(x_ij, y_ij);
                    Node_type type = i == (w / 2) && j == (h / 2) ? Node_type.BS : Node_type.Sensor;
                    if(type== Node_type.BS)
                    {
                        printfx("BS");
                    }
                    latex += create_hexagon(p, 6, r , type);
                }
            }

                return latex;
        }
        private string honeycomb(int w, int h, double r)
        {
            ns_point center = new ns_point(400, 100);
            string latex = "";
            double size = r;
            for (int j = 0; j < h; j++)
            {
                double hh = 2 * size;
                double hw = size * Math.Sqrt(3);
                ns_point fx = center;
                for (int i = 0; i < w; i++)
                {
                    //double shift = i == 0 ? hw / 2 - hw / 4 : 0;
                    //create_hexagon(fx.shift(shift, 0), 12, size);
                    latex += create_hexagon(fx, 6, size);


                    fx = fx.shift(hw, 0);
                }
                double x = hw * cos(60);
                double y = hw * sin(60);
                center = center.shift(-x, y);
            }
            // create_hexagon(center, 12, size);
            return latex;
        }
        private List<ns_point> create_gon(ns_point center , int n, double size)
        {
            List<ns_point> points = new List<ns_point>();
            ns_point p = new ns_point();
            for (int i = 1; i <= n; i++)
            {
                double angle = (360 / n) * i;
                var angle_rad = angle * Math.PI / 180;
                double x = center.x + size * Math.Cos(angle_rad);
                double y = center.y + size * Math.Sin(angle_rad);
                p = new ns_point(x, y);
                points.Add(p);
            }
            return points;
        }
        private string create_hexagon(ns_point center, int n, double size , Node_type type)
        {

            ns_pen pen = new ns_pen();
            ns_point p = new ns_point();
            List<ns_point> points = new List<ns_point>();

            //pen.M(center);
            //mcanvas.Children.Add(center.render(5));
            if(type == Node_type.Sensor)
            add_sensor(center);
            if (type == Node_type.BS)
                add_Bs(center);
            if (type == Node_type.MC)
                add_MC(center);
            for (int i = 0; i <= n; i++)
            {
                double angle = 60 * i - 30;
                var angle_rad = angle * Math.PI / 180;
                double x = center.x + size * Math.Cos(angle_rad);
                double y = center.y + size * Math.Sin(angle_rad);
                p = new ns_point(x, y);
                points.Add(p);
                if (i == 0)
                {
                    pen.M(p);
                    continue;
                }
                pen.L(p);
            }
            pen.Strokecolor = p.ColorFromPoint();
            pen.thickness = 1;
            //pen.FillColor = p.ColorFromPoint();
            Path pxp = pen.getPath();
            // pxp.Fill = p.ColorFromPoint();
            mcanvas.Children.Add(pxp);
            return hexaToLatex(points, center, type);
        }
        private string create_hexagon_withCircle(ns_point center, double size)
        {

            ns_pen pen = new ns_pen();
            ns_point p = new ns_point();
            List<ns_point> points = new List<ns_point>();

            //pen.M(center);
            //mcanvas.Children.Add(center.render(5));
            Path pp = center.shift(0.5*cos(60),  0.5*sin(60)).render(size);
            for (int i = 0; i <= 6; i++)
            {
                double angle = 60 * i - 30;
                var angle_rad = angle * Math.PI / 180;
                double x = center.x + size * Math.Cos(angle_rad);
                double y = center.y + size * Math.Sin(angle_rad);
                p = new ns_point(x, y);
                points.Add(p);
                if (i == 0)
                {
                    pen.M(p);
                    continue;
                }
                pen.L(p);
            }
            pen.Strokecolor = p.ColorFromPoint();
            pen.thickness = 1;
            //pen.FillColor = p.ColorFromPoint();
            Path pxp = pen.getPath();
            // pxp.Fill = p.ColorFromPoint();
            mcanvas.Children.Add(pxp);
            mcanvas.Children.Add(pp);
            return "";// hexaToLatex(points, center, Node_type.Sensor);
        }
        private string create_hexagon(ns_point center, int n, double size)
        {

            ns_pen pen = new ns_pen();
            ns_point p = new ns_point();
            List<ns_point> points = new List<ns_point>();

            //pen.M(center);
            //mcanvas.Children.Add(center.render(5));
            add_sensor(center);
            for (int i = 0; i <= n; i++)
            {
                double angle = 60 * i - 30;
                var angle_rad = angle * Math.PI / 180;
                double x = center.x + size * Math.Cos(angle_rad);
                double y = center.y + size * Math.Sin(angle_rad);
                p = new ns_point(x, y);
                points.Add(p);
                if (i == 0)
                {
                    pen.M(p);
                    continue;
                }
                pen.L(p);
            }
            pen.Strokecolor = p.ColorFromPoint();
            pen.thickness = 1;
            //pen.FillColor = p.ColorFromPoint();
            Path pxp = pen.getPath();
           // pxp.Fill = p.ColorFromPoint();
            mcanvas.Children.Add(pxp);
            return hexaToLatex(points, center, Node_type.Sensor);
        }
        private void add_to_nodelist(string id, ns_node node)
        {
            init_node_events(node, id);
            node.SetPrintfHandler(printfx);
            this.NodesList.Add(id, node);
        }
        private void OutputTcpPortFilters()
        {
            int jx = this.bs_count;
            int k = this.mc_count;
            int N = this.NodesList.Count;
            int i = 0;
            string fn = "port.dat";
            string filter = "";
            foreach(var node in this.NodesList)
            {
                int port = node.Value.getPort();
                string or = i == N - 1 ? "" : "or";
                filter += string.Format("tcp.port=={0} {1} ", port, or);
                i++;
            }
            File.WriteAllText(fn, "");
            File.WriteAllText(fn, filter);
        }
        private int getnet_id()
        {
            Random mx = new Random(DateTime.Now.Millisecond);
            int r = mx.Next(10, 500);
            int id =  1000 * this.bs_count + 100 * this.mc_count + 10 * this.sensor_count;

            return id;
        }
        private void add_Bs(ns_point location)
        {
            string id = string.Format("BS_{0}", bs_count + 1);
            base_station bs = new base_station(id, location);
            bs.OnPowerDown += sn_OnPowerDown;
            add_to_nodelist(id, bs);
            this.bs_count++;
            bs.SetNetId(getnet_id());
        }
        private void add_sensor(ns_point location)
        {
            string id = string.Format("S_{0}", this.sensor_count + 1);
            sensor_node sn = new sensor_node(id, location);
            sn.OnPowerDown += sn_OnPowerDown;
            add_to_nodelist(id, sn);

            this.sensor_count++;
            sn.SetNetId(getnet_id());
        }

        void sn_OnPowerDown(ns_node node)
        {
            //MessageBox.Show("Power Down :" + node.tag);
            printfx("{0} power down:{1}", node.tag, node.PowerConfig.BatteryCapacity);
            printfx(" life time:{0}", node.getLifeTime().ToString(@"dd\.hh\:mm\:ss"));
            node.Stop();

        }
        private void add_MC(ns_point location)
        {
            string id = string.Format("MC_{0}", this.mc_count + 1);
            mobile_charger mc = new mobile_charger(id, location);
            mc.OnPowerDown += sn_OnPowerDown;
            mc.OnNodeChargeDoneStats += mc_OnNodeChargeDoneStats;
            add_to_nodelist(id, mc);
            this.mc_count++;
            mc.SetNetId(getnet_id());
            mc.SetTravelHandler(MC_TravelHandler);
            this.MobileChargers.Add(id, mc);
        }

        
       private void MC_TravelHandler(mobile_charger mc)
        {
            //MessageBox.Show("OK");
            try
            {
                double bat_cap = mc.PowerConfig.BatteryCapacity;
                double thres = mc.PowerConfig.energy_threshold;
                if(mc.isBackhome() || bat_cap<=thres)
                {
                    printf("It's getting cold, I am coming home \n");
                    ns_Physics.goBackHome(mc, (s) => { printfx("{0}", s); }, (p) =>
                        {
                            this.Dispatcher.Invoke(() =>
                            {
                                Path mp = this.NodesList[mc.tag].getUiNode();
                                mp.Fill = Brushes.Red;
                                mp.Data = Geometry.Parse(p.GetRenderStr(mc.Radius));
                                // printfx("{0}", p);
                            });

                        });
                    return;
                }
                if (mc.MCStatus == Mc_Status.charging || mc.MCStatus == Mc_Status.travelling) return;
                ns_node nodetobeCharge = mc.getNodeToBeCharge();
                ns_point px = nodetobeCharge.location;
                mc.MCStatus = Mc_Status.travelling;
                
                
               
                //Path mp = mc.getUiNode();
                
                //this.Dispatcher.Invoke(() =>
                {

                    ns_Physics.MoveToPoint(mc, nodetobeCharge, (s) => { printfx("{0}", s); }, (p) =>
                    {


                        // mc.location = p;
                        //printfx("On MC_TravelHnadler");
                        //printfx("{0}",p);
                        this.Dispatcher.Invoke(() =>
                        {
                            Path mp = this.NodesList[mc.tag].getUiNode();
                            mp.Fill = Brushes.Red;
                            mp.Data = Geometry.Parse(p.GetRenderStr(mc.Radius));
                            // printfx("{0}", p);
                        });

                    });
                }
                // });
            }
            catch (Exception ex)
            {

               // MessageBox.Show(ex.Message);
                printfx("Ex:{0}", ex.Message);
            }


        }
        private void toLatex(List<ns_point> points , ns_point center)
        {
            string latex = "";

            
            int i = 0;
            foreach(var p in points)
            {
                string app = i < points.Count - 1 ? "--" : "";
                latex += string.Format("({0:0.00},{1:0.00}){2} ", (p.x / 50), (p.y / 50), app);
                i++;
            }
            latex = string.Format("\\draw {0};\n", latex);
            latex += string.Format("{0}\n", center.toLatexCircle(5, 50));
            File.AppendAllText("latex.tex", latex);
        }
        private void create_polygon(ns_point center , int n , double size)
        {
            ns_pen pen = new ns_pen();
            ns_point p = new ns_point();
            List<ns_point> points = new List<ns_point>();

            //pen.M(center);
           // mcanvas.Children.Add(center.render(5));
            for (int i = 0; i <= n; i++)
            {
                double angle = (360 / n) * i;// -(360 / 2 * n);
                var angle_rad = angle * Math.PI / 180;
                double x = center.x + size * Math.Cos(angle_rad);
                double y = center.y + size * Math.Sin(angle_rad);
                p = new ns_point(x, y);
                string id = string.Format("s_{0}{1}", this.i, i);

                add_sensor(p);
                points.Add(p);
                if (i == 0)
                {
                    pen.M(p);
                    continue;
                }
                pen.L(p);
            }
            pen.Strokecolor = p.ColorFromPoint();
            pen.thickness = 1;
            //pen.FillColor = p.ColorFromPoint();
            Path pxp = pen.getPath();
           // pxp.Fill = p.ColorFromPoint();
            mcanvas.Children.Add(pxp);
            //toLatex(points, center);
        }
        
        private void add_algorithm(string algo_type)
        {
            this.algo_type.Items.Add(algo_type);
        }
        private void SetTitle()
        {
            this.Title = this.File_name;
        }
        private void ClearAll()
        {
            this.mcanvas.Children.Clear();
            this.NodesList.Clear();
            this.MobileChargers.Clear();
            clear_combobox();
            this.screen.Text = "";
            this.i = 0;
            this.bs_count = 0;
            this.sensor_count = 0;
            this.mc_count = 0;
            
        }
        private ns_node _onNodeType(ns_node n )
        {
            if (n.NodeType == Node_TypeEnum.Sensor_node)
            {
                //add_sensor(fx.location);
                this.sensor_count++;
            }
            else if (n.NodeType == Node_TypeEnum.Mobile_Charger)
            {
                //add_MC(fx.location);
                this.mc_count++;
            }
            else if (n.NodeType == Node_TypeEnum.BaseStation)
            {
                //add_Bs(fx.location);
                this.bs_count++;
            }
            else
            {
                //add_sensor(fx.location);
                this.sensor_count++;
            }
            n.SetNetId(getnet_id());
            n.OnPowerDown += sn_OnPowerDown;
            n.SetPrintfHandler(printfx);
            add_node_combobox(n.tag);
            return n;

        }
        private void LoadNet(string fn)
        {
            ClearAll();
            Dictionary<string, bool> fx_dict = new Dictionary<string, bool>();
            Dictionary<string, ns_node> dict = new Dictionary<string, ns_node>();
            try
            {
                string data = File.ReadAllText(fn);
               
                dict = JsonConvert.DeserializeObject<Dictionary<string, ns_node>>(data);

                if (dict != null) 
                {
                    foreach(var node in dict)
                    {
                        ns_node fx = node.Value;
                        //printf("{0}", fx.tag);
                        ns_node n = new ns_node(fx.tag, fx.location);
                        n.NodeType = fx.NodeType;
                        if (fx.NodeType == Node_TypeEnum.Sensor_node)
                        {

                            this.sensor_count++;
                           
                         
                        }
                        else if (fx.NodeType == Node_TypeEnum.Mobile_Charger)
                        {

                            this.mc_count++;
                           
                        }
                        else if (fx.NodeType == Node_TypeEnum.BaseStation)
                        {

                            this.bs_count++;
                            
                        }
                        else
                        {
                            
                           this.sensor_count++;
                        }
                        n.SetNetId(getnet_id()); /// must be before you add links .

                       
                        if (fx.Links != null)
                        {
                            foreach (var a in fx.Links)
                            {
                                //printf("key :{0} \t value:{1}", a.key, a.value);
                                n.AddAttribute(a.key, a.value);

                            }
                        }
                        string cnodes = fx.JsonChildNodes;
                        foreach (var c in cnodes.Split(','))
                        {
                            if (string.IsNullOrEmpty(c)) continue;
                            if (dict.ContainsKey(c))
                            {
                                ns_node ff = dict[c];
                                double cost = n.getLinkCost(ff);
                                //printf("link cost : {0}-{1}:{2}", n.tag, ff.tag, cost);
                                n.addLink(ff, cost);
                            }
                        }

                      // n = _onNodeType(n);
                       // printf("---------------------");
                        n.PowerConfig = fx.PowerConfig;
                        n.OnPowerDown += sn_OnPowerDown;
                        n.SetPrintfHandler(printfx);
                        add_node_combobox(n.tag);
                        
                        n.__RenderTo(mcanvas.Children, fx_dict);
                        n.getUiNode().PreviewMouseDown += UiNode_PreviewMouseDown;
                        n.getUiNode().PreviewMouseRightButtonDown += ppx_PreviewMouseRightButtonDown;
                        if (!this.NodesList.ContainsKey(node.Key))
                        {
                            this.NodesList.Add(node.Key, n);
                        }
        
                      
                    }
                    this.i = dict.Count;
                    //dict[1].RenderTo(mcanvas.Children);
                }
            }
            catch (Exception ex)
            {
              // MessageBox.Show(ex.Message);
                throw ex;
            }
        }
        private void setLocation(Path mp, Point point)
        {
            string data = "M {0} {1} A 14 14 0 1 1 {0} {2} Z ";
            data = string.Format(data, point.X - 10, point.Y - 10, point.Y - 10 + 0.77);
            this.Dispatcher.Invoke(() =>
                {
                    mp.Data = Geometry.Parse(data);
                });
        }



        void s0_OnPowerReachThreshold(sensor_node sens)
        {

            //MessageBox.Show("Power is Down");
            this.Dispatcher.Invoke(() =>
                {
                  //  printf("node:{0} -->Eng:{1}", sens.tag, sens.Energy);
                    sens.Shutdown();
                });
           
        }
        private void printfx(string fmt , params object[] args)
        {
            this.Dispatcher.Invoke(() =>
                {
                    string msg = string.Format(fmt, args) + "\n";
                    File.AppendAllText(this.app_log_file, msg);
                    printf(fmt, args);
                });
        }
        private void simple_binary_tree( ns_node base_station , double shiftx, double shifty , int n  , int space)
        {
            //double x = 480, y = 140;
            //double shiftx = 100;
            //double shifty = 60;
            //int n = 6;
            //ns_node parent = new ns_node("p0", x, y);
            //int i = 0;
            //simple_binary_tree(parent, shiftx, shifty, n, i);
            if (n == 0) return;
            string leftx = string.Format("L{0}", space);
            string rightx = string.Format("R{0}", space);
            sensor_node left = new sensor_node(leftx, base_station.location.shift(-shiftx, shifty));
            sensor_node right = new sensor_node(rightx, base_station.location.shift(shiftx, shifty));
            base_station.addLink(left, n);
            base_station.addLink(right, n);
            space++;
            //shiftx -=  (n) * Math.Pow(2,  n-space + 1);
            n = n - 1;
            simple_binary_tree(left, shiftx, shifty, n , space);
            simple_binary_tree(right, shiftx, shifty, n , space);
            base_station.RenderTo(mcanvas.Children);
        }
       

        private UIElement getElementByName<T>(UIElementCollection eles , string name)
        {
           // UIElement ele = null;
            try
            {
                foreach (var e in eles)
                {
                    if (e.GetType() == typeof(T))
                    {
                        string e_name = e.GetType().GetProperty("Name").GetValue("") as string;
                        printf("name:{0}", e_name);

                        if (e_name == name)
                        {
                            return (UIElement)e;
                        }
                    }
                }
                return null;
            }
            catch (Exception exx)
            {
                
                throw exx;
            }
        }
        private Path getElementbyId(UIElementCollection eles , string id)
        {
            Path ele =null;;//= null;
            foreach(var e in eles)
            {
                if(e.GetType() == typeof(Path))
                {
                    ele = (Path)e;
                    if(ele.Name==id)
                    {
                        return ele;
                    }
                }
            }
            return ele;
        }
        private void Button_Click(object sender, RoutedEventArgs e)
        {

            clear();
            fx_node fxnode = new fx_node(mcanvas, 10, 10);

            


        }
        
      
       
        private bool contains(SortedSet<ns_node> nodes , ns_node node)
        {
           
            foreach(ns_node n in nodes)
            {
                if (n == node) return true;
            }
            return false;
        }
        private void Forward_packet(List<ns_node> nodes)
        {
            ns_node first = nodes[0];
            ns_node last = nodes.Last();
            string msg = string.Format("msg from {0} to {1}", first.tag, last.tag);
            ns_node dst = last;
            for (int i = 0; i < nodes.Count; i++) 
            {
                if(i==0)
                {
                    first = nodes[i];
                    continue;
                }
                last = nodes[i];
                first.SendToClient(last, msg);
                first = last;
                if (first.tag == dst.tag) break;
            }
        }
        private void Draw(List<ns_node> nodes)
        {
            if(nodes == null)
            {
                printf("No path");
                return;
            }
            ns_pen pen = new ns_pen();
            pen.Strokecolor = Brushes.Green;
            pen.thickness = 2.1;
            int i = 0;
            int count = nodes.Count;
            printf("hop Count :{0}", nodes.Count);
            double sum = 0;
           // string c = "";
            //ns_node nn = nodes.First();
            ns_point px = new ns_point();
            ns_node prev = nodes[0];
            foreach (ns_node node in nodes)
            {
                printf("{0}", node.tag);
                if (i == 0)
                {
                    i++;
                    pen.M(node.location);
                    prev = node;
                    continue;
                }
               pen.L(node.location);
               px += node.location;
               // pen.data+=string.Format("C")
               // c += string.Format("{0} {1} {2}", node.location.x, node.location.y, i != count - 1 ? "," : "");
               // if()
               sum += prev.getLinkCost(node);
               prev = node;
                i++;
            }
          //  pen.data += string.Format("C{0}", c);
            Path pp = pen.getPath();
            string src_id = src_node_box.SelectedItem as string;
            string sink_id = sink_node_box.SelectedItem as string;
            string a_type = algo_type.SelectedItem as string;
            printf("Path Cost:{0}", sum);
            pp.Stroke = px.ColorFromPoint();
            pp.ToolTip = string.Format("src :{0} , sink:{1} a_type:{2} , total cost :{3}", src_id, sink_id, a_type,sum);
            mcanvas.Children.Add(pp);
            //Forward_packet(nodes);
        }
        private void Draw(SortedSet<ns_node> nodes)
        {
            ns_pen pen = new ns_pen();
            pen.Strokecolor = Brushes.Green;
            pen.thickness = 2.1;
            int i = 0;
            //ns_node nn = nodes.First();
            foreach(ns_node node in nodes)
            {
                if(i==0)
                {
                    i++;
                    pen.M(node.location);
                    continue;
                }
                pen.L(node.location);
                
                i++;
            }
            mcanvas.Children.Add(pen.getPath());
        }
      
      
      
        private void printf(string fmt, params object[] params_)
        {
            string s = string.Format(fmt, params_);
            logmsg(s);
        }
        
        private void logmsg(string msg)
        {
            string txt = screen.Text;
            if (txt.Length >= this.max_screen_buffer_len)
                screen.Text = "";
            screen.Text += msg + Environment.NewLine; ;
        }
       
    

        private void clear()
        {
            i = 0;
            if (mcanvas.Children.Count > 0)
            {
                this.Dispatcher.Invoke(() =>
                {
                    mcanvas.Children.Clear();
                    screen.Text = "";
                });

            }

        }
       
        private void mcanvas_MouseDown(object sender, MouseButtonEventArgs e)
        {
          //  
            if (e.LeftButton == MouseButtonState.Pressed)
            {
                
            }
            //MessageBox.Show("clc");
        }

        private void mcanvas_MouseLeave(object sender, MouseEventArgs e)
        {
            i = 0;
            this.Dispatcher.Invoke(() =>
            {
                ns_pen pen = new ns_pen(this.data);
                pen.Strokecolor = Brushes.Red;
                pen.thickness = 1.55;
                mcanvas.Children.Add(pen.getPath());
                data = "";
                i = 0;
                //MessageBox.Show("leave");
            });
        }
        private string data = "";
        private void mcanvas_MouseMove(object sender, MouseEventArgs e)
        {
           // MessageBox.Show("iiss");
           // if (i > 0)
            {
                ns_point p = new ns_point( e.GetPosition(null));
                if (i == 0)
                {

                    data += p.moveto();
                    //return;
                }
                data += p.lineto();
               // mcanvas.Children.Add(p.render(1));
                i++;
            }
        }

        private void Grid_MouseDown(object sender, MouseButtonEventArgs e)
        {

        }
        
        //int i = 0;
        private void add_node_combobox(string node)
        {
            this.src_node_box.Items.Add(node);
            this.sink_node_box.Items.Add(node);
        }
        private void clear_combobox()
        {
            this.src_node_box.Items.Clear();
            this.sink_node_box.Items.Clear();
        }
        private void init_node_events(ns_node node  , string id )
        {
            node.RenderTo(mcanvas.Children);
            Path ppx = node.getUiNode();
            ppx.PreviewMouseDown += UiNode_PreviewMouseDown;

            ppx.PreviewMouseRightButtonDown += ppx_PreviewMouseRightButtonDown;

            //Wind_node_params wind_params = new Wind_node_params(node , this);
            add_node_combobox(id);
        }
        private void add_nodex(string id , ns_point p)
        {
            ns_node node = new ns_node(id, p);
            this.last_added = id;
           

            node.RenderTo(mcanvas.Children);



            Path ppx = node.getUiNode();
            ppx.PreviewMouseDown += UiNode_PreviewMouseDown;

            ppx.PreviewMouseRightButtonDown += ppx_PreviewMouseRightButtonDown;

            //Wind_node_params wind_params = new Wind_node_params(node , this);
            add_node_combobox(id);
            if (!this.NodesList.ContainsKey(id))
            {
                this.NodesList.Add(id, node);
            }
        }
        private void mcanvas_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
          //  MessageBox.Show("pp");
           if(e.LeftButton== MouseButtonState.Pressed && !BoolUsePen )
           {
               string id = string.Format("n_{0}", i + 1);
               ns_point p = new ns_point(e.GetPosition(null));
               if(this.NodeType == Node_type.Sensor)
               {
                   add_sensor(p); // for quick and mostly added .
               }
               if (this.NodeType == Node_type.MC)
               {
                   add_MC(p);
               }

               else if (this.NodeType == Node_type.BS)
               {
                   // add_sensor(p);
                   add_Bs(p);
               }
               else ;
               //add_nodex(id, p);
               
               
               last = p;
               i++;
           }
        }

        void ppx_PreviewMouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            Path pp = sender as Path;
            ns_node node = pp.Tag as ns_node;
            //Wind_node_params wind = new Wind_node_params(node, this);
            //
            //wind.ShowDialog();
            Wind_Node_Config config = new Wind_Node_Config(node, this);
            config.ShowDialog();

        }
        
        private void UiNode_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            if (!BoolUsePen) return;
            Path mp = sender as Path;
           // MessageBox.Show(mp.Name);
            //Random r = new Random(DateTime.Now.Millisecond);
            double cost = 0;// r.Next(50);
            if(j==0)
            {
                prev_clicked_node = mp.Name;
                j++;
                return;
            }
            if(j==1)
            {
                j= 0;
                if (prev_clicked_node == mp.Name) return; // No ele
                ns_node prev = NodesList[prev_clicked_node];
                ns_node current = NodesList[mp.Name];
                prev.addLink(current, cost);
                cost = prev.mdistance(current);
                ns_link.RenderLink(prev, current, cost, mcanvas.Children);
                //Path pp = ns_line.Arrow(prev.location, current.location, 1);
                //mcanvas.Children.Add(pp);
                prev_clicked_node = "";
            }
        }

        private void stop_Click(object sender, RoutedEventArgs e)
        {
            this.mcanvas.PreviewMouseDown -= mcanvas_PreviewMouseDown;
        }

        private void UsePen_Click(object sender, RoutedEventArgs e)
        {
            this.mcanvas.PreviewMouseDown -= mcanvas_PreviewMouseDown;
            this.BoolUsePen = true;
            ConnectAll();
        }
        
        private void AddNode_Click(object sender, RoutedEventArgs e)
        {
            Button bt = sender as Button;
            this.mcanvas.PreviewMouseDown -= mcanvas_PreviewMouseDown;
            this.mcanvas.PreviewMouseDown += mcanvas_PreviewMouseDown;
            this.BoolUsePen = false;
            string type = bt.Tag as string;
            if (type == "sensor") 
            {
                this.NodeType = Node_type.Sensor;
                return;
            }
            if (type == "mc")
            {
                this.NodeType = Node_type.MC;
                return;
            }
            if (type == "bs")
            {
                this.NodeType = Node_type.BS;
                return;
            }
        }

        private void mcanvas_PreviewKeyDown(object sender, KeyEventArgs e)
        {
           if(e.Key ==( Key.Space))
           {
               MessageBox.Show("control Z");
           }
        }
        private void toLatexTikz(string latex , string fn )
        {
            string header = "\\documentclass[border=1mm]{article}\n";
            header += "\\usepackage{tikz}\n";
            header += "\\begin{document}\n";
            header += "\\begin{tikzpicture}\n";
            latex += "\n \\end{tikzpicture}";
            latex += "\n \\end{document}";
            File.WriteAllText(fn, header + "\n" + latex);
        }
        private void NodeList2Latex(string fn)
        {
            string latex = "";

            Dictionary<string, bool> mdic = new Dictionary<string, bool>();

            string header = "\\documentclass[border=1mm]{article}\n";
            header+="\\usepackage{tikz}\n";
            header+="\\begin{document}\n";
            header += "\\begin{tikzpicture}\n";
            foreach(var n in this.NodesList)
            {
                ns_node node = n.Value;
                latex += node.toLatex(mdic);
            }
            latex += "\n \\end{tikzpicture}";
            latex += "\n \\end{document}";
            File.WriteAllText(fn, header + "\n" + latex);
        }
        private void  SaveAs_Latex_Click(object snd , RoutedEventArgs e )
        {
            if (this.NodesList.Count == 0) return;
            SaveFileDialog sf = new SaveFileDialog();
            sf.AddExtension = true;
            sf.InitialDirectory = getCurDir();
            sf.DefaultExt = "tex";
            if (sf.ShowDialog() == true)
            {
                string fn = sf.FileName;
                
                //this.File_name = fn;
               // SetTitle();
                try
                {
                   // string json = JsonConvert.SerializeObject(this.NodesList);
                    NodeList2Latex(fn);
                }
                catch (Exception ex)
                {
                    //printf("Exe:{0}", ex.Message);
                    MessageBox.Show(ex.Message);
                }
            }
        }
        private void SaveNetwork(string fn)
        {
            if (this.NodesList.Count == 0) return;
            if (this.File_name != string.Empty) return;
            try
            {
                this.File_name = fn;
                SetTitle();
                string json = JsonConvert.SerializeObject(this.NodesList);
                File.WriteAllText(fn, json);
            }
            catch (Exception ex)
            {
                //printf("Exe:{0}", ex.Message);
                MessageBox.Show(ex.Message);
            }
        }
        private void SaveAs_topo_Click(object sender, RoutedEventArgs e)
        {

            if (this.NodesList.Count == 0) return;
            SaveFileDialog sf = new SaveFileDialog();
            sf.AddExtension = true;
            sf.InitialDirectory = getCurDir();
            sf.DefaultExt = "json";
            if (sf.ShowDialog() == true)
            {
                string fn = sf.FileName;
                this.File_name = fn;
                SetTitle();
                try
                {
                    string json = JsonConvert.SerializeObject(this.NodesList);
                    File.WriteAllText(fn, json);
                }
                catch (Exception ex)
                {
                    //printf("Exe:{0}", ex.Message);
                    MessageBox.Show(ex.Message);
                }
            }
        }
        private void Save_topo_Click(object sender, RoutedEventArgs e)
        {

            if (this.NodesList.Count == 0) return;
            if(string.IsNullOrEmpty(this.File_name))
            {
                SaveAs_topo_Click(sender, e);
                return;
            }
            {
                string fn = this.File_name;
                //this.File_name = fn;
                //SetTitle();
                try
                {
                    string json = JsonConvert.SerializeObject(this.NodesList);
                    File.WriteAllText(fn, json);
                }
                catch (Exception ex)
                {
                    //printf("Exe:{0}", ex.Message);
                    MessageBox.Show(ex.Message);
                }
            }
        }
        public static string getCurDir()
        {
            return Directory.GetCurrentDirectory();
        }
        public static string getParentDir(string dir)
        {
            return Directory.GetParent(dir).FullName;
        }
        public static string getAppRootDir()
        {
            return Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName;
        }
        private void load__net_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                OpenFileDialog fd = new OpenFileDialog();
                fd.InitialDirectory = getCurDir();
                fd.Title = "Choose network ";
                //"Image files (*.png;*.jpeg)|*.png;*.jpeg|All files (*.*)|*.*";
                fd.Filter = "Network Simulator (*.json;*.ns)|*.ns;*.json| All files (*.*)|*.*";
                if (fd.ShowDialog() == true)
                {
                    this.File_name = fd.FileName;
                    SetTitle();
                    LoadNet(fd.FileName);
                }
            } catch(Exception ex)
            {
               MessageBox.Show(ex.Message);
               
            }
        }

        private void new_net_Click(object sender, RoutedEventArgs e)
        {
            this.mcanvas.Children.Clear();
            this.NodesList.Clear();
            this.i = 0;
            this.File_name = "";
            this.screen.Text = "";
            clear_combobox();
            ClearAll();
            StopSimulation_Click(null, null);
            create_new_net();
        }

        private void gui_clear_Click(object sender, RoutedEventArgs e)
        {
            this.mcanvas.Children.Clear();
            this.NodesList.Clear();
            this.i = 0;
            this.mc_count = 0;
            this.sensor_count = 0;
            this.bs_count = 0;
            this.File_name = "";
            this.screen.Text = "";
            this.NodeType = Node_type.Sensor;
            clear_combobox();
            ClearAll();
            StopSimulation_Click(null, null);

        }
        private string last_added = "";
        private void redo_Click(object sender, RoutedEventArgs e)
        {
            if (this.NodesList.Count > 0 && !string.IsNullOrEmpty(this.last_added))
            {
               
                ns_node node = this.NodesList.Values.Last();
                this.NodesList.Remove(node.tag);
                i = i - 1;
                Path mp = getElementbyId(mcanvas.Children, this.last_added);
                
                mcanvas.Children.Remove(node.getUiNode());
                mcanvas.Children.Remove(node.getNodeText());
                src_node_box.Items.RemoveAt(i);
                sink_node_box.Items.RemoveAt(i);
                
            }
        }

        private void do_button_Click(object sender, RoutedEventArgs e)
        {

        }
        private bool is_sim_start = false;
        private void find__node_Click(object sender, RoutedEventArgs e)
        {
            if(this.NodesList.Count==0) return;
            string a_type = algo_type.SelectedItem as string;
            
            //if(!is_sim_start)
            //{
            //    MessageBox.Show("You should first start Simulation !");
            //    return;
            //}
            
            if(a_type == "assign_threshold")
            {
                assign_threshold();
                return;
            }
            string src_id = src_node_box.SelectedItem as string;
            string sink_id = sink_node_box.SelectedItem as string;
            
            if (a_type == "Dijkstra_Connect")
            {
                ConnectAll();
                return;
            }
            if(src_id==sink_id) return;
            ns_node src = this.NodesList[src_id];
            ns_node sink = this.NodesList[sink_id];

            printf("src:{0} dst:{1} a:{2}", src_id, sink_id, a_type);
            //add_algorithm("dijkstra");
            //add_algorithm("A Star");
            //add_algorithm("Bread-first search");
            //add_algorithm("greedy_best_first_search");
            if (a_type == "send packet")
            {
                net_packet packet = new net_packet(src_id, sink_id, packetType.Data, sink_id);
                packet.end_src = src_id;
                src.ns_send_packet(packet);
                //src.SendPacketTo(sink, "Hello, what's up");
                return;
            }
            if (a_type == "dijkstra")
            {
                graph_algorithms.printf = printf;
               List<ns_node> path =  graph_algorithms.Dijkstra(src, sink);
               Draw(path);
                return;
            }
            if (a_type == "A Star")
            {
                graph_algorithms.printf = printf;
                List<ns_node> path = graph_algorithms.Astar(src, sink, (n1, n2) =>
                    {
                        return Math.Abs(n1.getEn() - n2.getEn());
                    });
                Draw(path);
                return;
            }
            if (a_type == "Bread-first search")
            {
                graph_algorithms.printf = printf;
                List<ns_node> path = graph_algorithms.breadth_first_search(src, sink);
                Draw(path);
                return;
            }
            if (a_type == "greedy_best_first_search")
            {
                graph_algorithms.printf = printf;
                List<ns_node> path = graph_algorithms.greedy_best_first_search(src, sink);
                Draw(path);
                return;
            }
            if (a_type == "hueristic_best_first_search")
            {
                graph_algorithms.printf = printf;
                List<ns_node> path = graph_algorithms.hueristic_best_first_search(src, sink, (n1, n2) =>
                    {
                        return n1.mdistance(n2);
                    });
                Draw(path);
                return;
            }
            if( a_type == "MC_FIND")
            {
                graph_algorithms.printf = printf;
                List<ns_node> path = graph_algorithms.Dijkstra_GetNearestMC(src);
                Draw(path);
                return;
            }
            if(a_type =="Clustering")
            {
                Find_NeartestNodesTo(src);
            }
            
        }

        private void StartNetwork_Click(object sender, RoutedEventArgs e)
        {
            if (this.NodesList.Count == 0) return;
            this.mcanvas.PreviewMouseDown -= mcanvas_PreviewMouseDown;
            this.BoolUsePen = false;
            this.is_sim_start = true;
            string fn = string.Format("net_{0}.json", DateTime.Now.ToString("yyyyMMdd_hhmmss"));
            eng_threshold_method method = AppConfig.getThresholdMethod();
            printf("Threshold method:{0}", method.ToString());
            ConfigNodes();
            SaveNetwork(fn);
            
            foreach(var fx in this.NodesList)
            {
                ns_node node = fx.Value;
                if(node.NodeType == Node_TypeEnum.Sensor_node)
                {
                    sensor_node sn = node as sensor_node;
                    sn.SetMobileChargers(this.MobileChargers);
                   // printf("Mobile charges set for:{0}\n", sn.tag);
                }
                node.init_com_Module();
                node.Start();
                
            }
            Random time = new Random(DateTime.Now.Millisecond);
            double[] win = guassian_window(this.NodesList.Count, 0.3, 120);
            int i = 0;
            foreach(var fx in this.NodesList)
            {
                fx.Value.SetBaseStation(this.getBS());
                fx.Value.StartNetTimer(win[i] + 60);
                i++;
            }
            
             init_sim_timer();
             // we should wait until the network start receiving packets.
            if(method == eng_threshold_method.assign_threshold_algorithm)
            {
                //printf("assigning threshold based on routing algorithm");
                //assign_threshold();
            }
             
            
        }

        private void StopSimulation_Click(object sender, RoutedEventArgs e)
        {
            if (this.NodesList.Count == 0) return;
            this.is_sim_start = false;
            foreach (var fx in this.NodesList)
            {
                ns_node node = fx.Value;
               
                node.Stop();
            }
        }
       

        private void show_stats_Click(object sender, RoutedEventArgs e)
        {
           
            //this.net_stats_wind = new Wind_net_Stat(this);
            net_stats_wind mwin = new net_stats_wind(this, this.NodesList);
            mwin.ShowWind();
            
        }

        private void ExportPower_Click(object sender, RoutedEventArgs e)
        {
            if (this.NodesList.Count == 0) return;
            //if (!this.is_sim_start) return;
            string date = DateTime.Now.ToString("hhmmss");
            string fn_name = string.Format("profile_{0}.dat", date);
            string data = "";
            int sn = 1 ;
            string[] files = new string[this.NodesList.Count];
            int i =0;
            foreach(var kvp in this.NodesList)
            {
                var node = kvp.Value;
                string fn = string.Format("eng_{0}.dat", node.tag);
                //files[i] = fn;
                i++;
               
                if (node.NodeType != Node_TypeEnum.Sensor_node) continue;
               
                double mean = DataLoader.GetMean(fn, true);
                data += string.Format("{0}\n",mean);
                sn++;

            }
            File.WriteAllText(fn_name, "");
            File.WriteAllText(fn_name, data);
            /*string out_fn = string.Format("nodes_power_{0}.dat", date);
            DataLoader.ConcatFilesInto(files, out_fn);*/
        }
        
    }
}
