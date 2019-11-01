package org.fog.entities;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.apache.commons.math3.genetics.Population;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.CMAESOptimizer.PopulationSize;
import org.apache.commons.math3.util.Pair;
import org.cloudbus.cloudsim.Vm;
import org.cloudbus.cloudsim.core.CloudSim;
import org.cloudbus.cloudsim.core.SimEvent;
import org.cloudbus.cloudsim.power.PowerDatacenterBroker;
import org.cloudbus.cloudsim.power.PowerHost;
import org.fog.application.AppModule;
import org.fog.test.perfeval.VRGameFog;
import org.fog.utils.FogEvents;
import org.fog.utils.NetworkUsageMonitor;
import org.fog.utils.TimeKeeper;

public class FogBroker extends PowerDatacenterBroker
{
	protected List <Pair<Tuple,Double>> Tqueue;
	List<FogDevice> fogdevices1;
	protected Map<String, List<String>> appToModulesMap;
	protected HashMap<Tuple, Long> TupleIdToCpuLength;
	protected HashMap<Tuple, Long> TupleIdToCpuLength1;
	protected double uplinkLatency=6.0;
	int tries=0;
	int populationsize=40;
	private double cross_over_rate=0.6;
	private double mutation_rate=0.1;
	//int init_host1[][]=new int[10][vmlist.size()];
	//int cross_host[][]=new int[10][vmlist.size()];
	double prev_best=Double.MAX_VALUE;
	int count=0;
	public double getUplinkLatency() {
		return uplinkLatency;
	}



	public void setUplinkLatency(double uplinkLatency) {
		this.uplinkLatency = uplinkLatency;
	}



	public FogBroker(String name) throws Exception {
		super(name);
		Tqueue=new LinkedList<Pair<Tuple, Double>>();
		List<FogDevice> fogdevices1;
		appToModulesMap = new HashMap<String, List<String>>();
		TupleIdToCpuLength=new HashMap<Tuple, Long>();
		TupleIdToCpuLength1=new HashMap<Tuple, Long>();
		setUplinkLatency(uplinkLatency);
		// TODO Auto-generated constructor stub
		
	}

	

	@Override
	public void startEntity() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void processEvent(SimEvent ev) {
		// TODO Auto-generated method stub
		switch(ev.getTag())	
		{
		case FogEvents.TUPLE_ARRIVAL: 
			processTupleArrival(ev);
			break;
		case FogEvents.TUPLE_FINISHED: 
			processTupleFinished();
			break;
			
		}	
	}

	private void processTupleArrival(SimEvent ev) {
		
		
		Tuple tuple=(Tuple) ev.getData();
		TimeKeeper.getInstance().getTupleAssignedToFbroker().put(tuple.getActualTupleId(), CloudSim.clock());
		VRGameFog gameRes = new VRGameFog();
		List<FogDevice> fogdevices = gameRes.getFogDevices();
		//List<Sensor> sensor=gameRes.getSensors();
		List<Actuator> actuators=gameRes.getActuators();
		//System.out.println("Avilale Fog Devices are: ");
		//for(FogDevice f: fogdevices)
		//{
			//System.out.println(f.getName());
		//}
		//List<Vm> vm=new ArrayList<Vm>();
		Tqueue.add(new Pair<Tuple, Double>(tuple, CloudSim.clock()));
		if(Tqueue.size()>=10)
		{
		//random_cal(Tqueue, fogdevices,actuators);
			int[] allocation=host_genetic(Tqueue, fogdevices, actuators);
			for(int i=0; i<Tqueue.size();i++)
			{
				//String srcModule=(tqueue2.get(i).getKey()).getSrcModuleName();
				if((Tqueue.get(i).getKey()).getDirection()==Tuple.ACTUATOR)
				{
					int deviceId=(Tqueue.get(i).getKey()).getSourceDeviceId();					
					FogDevice fg=null;
					//System.out.println("DeviceId :"+deviceId);
					for(FogDevice f: fogdevices1)
					{
						if(f.getId()==deviceId)
						{					
							fg=(FogDevice)CloudSim.getEntity(deviceId);							
							break;
						}
					}
					for(Pair<Integer, Double> actuatorAssociation : fg.getAssociatedActuatorIds())
					{
						int actuatorId = actuatorAssociation.getFirst();
						double delay = actuatorAssociation.getSecond();
						String actuatorType = ((Actuator)CloudSim.getEntity(actuatorId)).getActuatorType();
						if((Tqueue.get(i).getKey()).getDestModuleName().equals(actuatorType))
						{
							send(actuatorId, delay, FogEvents.TUPLE_ARRIVAL, Tqueue.get(i).getKey());
						}
					}
				}
				else
				{
			
				TimeKeeper.getInstance().getTupleAssignedToFdeviceFromFbroker().put(Tqueue.get(i).getKey().getActualTupleId(), CloudSim.clock());
				//System.out.println(TimeKeeper.getInstance().getTupleAssignedToFdeviceFromFbroker());
				send(allocation[i], 0, FogEvents.TUPLE_ARRIVAL, Tqueue.get(i).getKey());
				NetworkUsageMonitor.sendingTuple(getUplinkLatency(),(Tqueue.get(i).getKey()).getCloudletFileSize());
			}
			
		
			Tqueue.clear();
		
			}}	
	}
		
		private void Sort(List<Pair<Tuple, Double>> tqueue2, List<FogDevice> fogdevices)
		{
			
			for(int i=0;i<tqueue2.size();i++)
			{
				if((tqueue2.get(i).getKey()).getDirection()==Tuple.ACTUATOR){
				int deviceId=(tqueue2.get(i).getKey()).getSourceDeviceId();
				FogDevice fg=null;
				//System.out.println("DeviceId :"+deviceId);
				for(FogDevice f: fogdevices)
				{
					if(f.getId()==deviceId)
					{
						fg=(FogDevice)CloudSim.getEntity(deviceId);
						break;
					}
				}
				for(Pair<Integer, Double> actuatorAssociation : fg.getAssociatedActuatorIds()){
					int actuatorId = actuatorAssociation.getFirst();
					double delay = actuatorAssociation.getSecond();
					String actuatorType = ((Actuator)CloudSim.getEntity(actuatorId)).getActuatorType();
					if((tqueue2.get(i).getKey()).getDestModuleName().equals(actuatorType)){
						send(actuatorId, delay, FogEvents.TUPLE_ARRIVAL, tqueue2.get(i).getKey());
						}
				}
				}
			else
			{
			long cpuLength=(tqueue2.get(i).getKey()).getCloudletLength();
			//int tupleId=(tqueue2.get(i).getKey()).getActualTupleId();
			TupleIdToCpuLength.put(tqueue2.get(i).getKey(), cpuLength);
			}
			
		}
		TupleIdToCpuLength1=sortByValue(TupleIdToCpuLength);
		//for(Map.Entry<Tuple, Long> e: TupleIdToCpuLength1.entrySet())
		//{
			//System.out.println("Tuple Id: " + e.getKey().getActualTupleId()+" Tuple length: "+e.getValue());
		//}
		SJF(TupleIdToCpuLength1,fogdevices,tqueue2);
		//System.out.println(TupleIdToCpuLength1);
		TupleIdToCpuLength1.clear();
		tqueue2.clear();
	}
		 private void SJF(HashMap<Tuple, Long> tupleIdToCpuLength12, List<FogDevice> fogdevices, List<Pair<Tuple, Double>> tqueue2) {
			// TODO Auto-generated method stub
			 //System.out.print(tupleIdToCpuLength12.size());
			 for(Map.Entry<Tuple, Long> ele:tupleIdToCpuLength12.entrySet())
			 {
				 Tuple id= (Tuple) ele.getKey();
				//System.out.println(id);
				 String module=id.getDestModuleName();
				 FogDevice f1=findFogWithModule(fogdevices, module);
				 //System.out.println("Selected fog device is: "+f1.getName());
				 send(f1.getId(), 0, FogEvents.TUPLE_ARRIVAL, id);
				 NetworkUsageMonitor.sendingTuple(getUplinkLatency(),id.getCloudletFileSize());
				 //System.out.println(module);
				/* Random r=new Random();
					int fg=r.nextInt(fogdevices.size());
					FogDevice fog1=fogdevices.get(fg);
					appToModulesMap=((FogDevice) fog1).getAppToModulesMap();
					//System.out.print(fog1.getName());
					for (Map.Entry<String, List<String>> me :appToModulesMap.entrySet()) {
						 //String key = me.getKey();
						  List<String> valueList = me.getValue();
						  //System.out.println("Key: " + key);
						  //System.out.print("Values: ");
						  for (String s : valueList) {
						    //System.out.print(s + " ");
							  if(s.equals(module))
							  {
								  //System.out.print("I am in Module"+module);
								  TimeKeeper.getInstance().getTupleAssignedToFdeviceFromFbroker().put(id.getActualTupleId(), CloudSim.clock());
								  send(fog1.getId(), 0, FogEvents.TUPLE_ARRIVAL, id);
								  NetworkUsageMonitor.sendingTuple(getUplinkLatency(), id.getCloudletFileSize());
						  }
							  else 
									continue;
						  }
						 // System.out.print(tqueue2.size());
					}*/
				
			 }
			 tupleIdToCpuLength12.clear();
			 
			
		}

		 public static HashMap<Tuple, Long> sortByValue(HashMap<Tuple, Long> hm) 
		 { 
	        // Create a list from elements of HashMap 
	        List<Map.Entry<Tuple, Long> > list = new LinkedList<Map.Entry<Tuple, Long> >(hm.entrySet()); 
	         // Sort the list 
	        Collections.sort(list, new Comparator<Map.Entry<Tuple, Long> >() { 
	            public int compare(Map.Entry<Tuple, Long> o1,  
	                               Map.Entry<Tuple, Long> o2) 
	            { 
	                return (o1.getValue()).compareTo(o2.getValue()); 
	            } 
	        });
	        HashMap<Tuple, Long> temp = new LinkedHashMap<Tuple, Long>(); 
	        for (Map.Entry<Tuple, Long> aa : list) { 
	            temp.put(aa.getKey(), aa.getValue()); 
	        } 
	        return temp; 
		 }

		@SuppressWarnings("null")
		private void random_cal(List<Pair<Tuple, Double>> tqueue2, List<FogDevice> fogdevices1, List<Actuator> actuators1) 
		{
			/*System.out.println("Fog Devices IDs are..-");
			System.out.println("");
			for(int l=0;l<fogdevices1.size();l++)		
			{
				System.out.print("\t  "+  fogdevices1.get(l).getId());
			}*/
			
		// TODO Auto-generated method stub
			for(int i=0; i<tqueue2.size();i++)
			{
				//String srcModule=(tqueue2.get(i).getKey()).getSrcModuleName();
				if((tqueue2.get(i).getKey()).getDirection()==Tuple.ACTUATOR)
				{
					int deviceId=(tqueue2.get(i).getKey()).getSourceDeviceId();					
					FogDevice fg=null;
					//System.out.println("DeviceId :"+deviceId);
					for(FogDevice f: fogdevices1)
					{
						if(f.getId()==deviceId)
						{					
							fg=(FogDevice)CloudSim.getEntity(deviceId);							
							break;
						}
					}
					for(Pair<Integer, Double> actuatorAssociation : fg.getAssociatedActuatorIds())
					{
						int actuatorId = actuatorAssociation.getFirst();
						double delay = actuatorAssociation.getSecond();
						String actuatorType = ((Actuator)CloudSim.getEntity(actuatorId)).getActuatorType();
						if((tqueue2.get(i).getKey()).getDestModuleName().equals(actuatorType))
						{
							send(actuatorId, delay, FogEvents.TUPLE_ARRIVAL, tqueue2.get(i).getKey());
						}
					}
				}
				else
				{						
				String  module=(tqueue2.get(i).getKey()).getDestModuleName();
				FogDevice f1=findFogWithModule(fogdevices1, module);
				//System.out.println("Selected fog device is: "+f1.getName());
				TimeKeeper.getInstance().getTupleAssignedToFdeviceFromFbroker().put(tqueue2.get(i).getKey().getActualTupleId(), CloudSim.clock());
				//System.out.println(TimeKeeper.getInstance().getTupleAssignedToFdeviceFromFbroker());
				 send(f1.getId(), 0, FogEvents.TUPLE_ARRIVAL, tqueue2.get(i).getKey());
				 NetworkUsageMonitor.sendingTuple(getUplinkLatency(),(tqueue2.get(i).getKey()).getCloudletFileSize());
				}
				
					//fogdevices1.clear();		  
					  
					  }
			tqueue2.clear();
		}
	
		 
		public int[] host_genetic(List<Pair<Tuple, Double>> tqueue2, List<FogDevice> fogdevices1, List<Actuator> actuators1)
		{
			
			this.Tqueue=tqueue2;
			this.fogdevices1=fogdevices1;
	        double starttime=java.lang.System.currentTimeMillis();
			int[][] initial_population=initialPop(tqueue2, fogdevices1);
			int[] best_set=new int[tqueue2.size()];
			double[] initial_fitness=fitnessCalc(initial_population);
			do
			{
				int[][] population=selection(initial_population);
				int[][] cross_population=cross_selection(population);
				int[][] mutate_pop=mutate(cross_population);
				double[] fitness=fitnessCalc(mutate_pop);
				int sum=0;
				for(int i=0;i<fitness.length;i++)
				{
					if(fitness[i]>initial_fitness[i])
						sum++;
				}
				//if(sum>(0.7*populationsize))
				//	initial_population=mutate_pop;
				fitness=null;
				tries++;
			}while(tries<2000);
	                double endtime=java.lang.System.currentTimeMillis();
	                 try{
	                            File fout = new File("D://GaTime.txt");
	                            FileOutputStream fos = new FileOutputStream(fout);
	                            java.io.BufferedWriter bw = new java.io.BufferedWriter(new java.io.OutputStreamWriter(fos));
	                            bw.write("something"+ (endtime-starttime) + "tries" + tries);
	                            bw.close();
	                            }
	                            catch(Exception e){}
			double[] final_fitness=fitnessCalc(initial_population);
			double min=final_fitness[0];
			int best_id=0;
			for(int i=0;i<final_fitness.length;i++)
			{
				if(final_fitness[i]<min)
				{
					min=final_fitness[i];
					best_id=i;
				}
			}
			best_set=initial_population[best_id];
	                String s="Gbest Solution is"+"\n";
	                for(int p=0;p<best_set.length;p++)
	                    s=s+".."+best_set[p];
	                try{
	                        FileOutputStream fout=new FileOutputStream("D://GAPOP.txt");
	                        fout.write(s.getBytes());
	                        fout.close();
	                   }catch(IOException ex){}

			return best_set;
		}
		/*
		 * INITIAL POPULATION GENERATION
		 */
		public int[][] initialPop(List<Pair<Tuple, Double>> tqueue2, List<FogDevice> fogdevices1)
		{
			int[][] initial_population=new int[populationsize][this.Tqueue.size()];
			Random r=new Random();
			
			int[] fogDevId=new int[fogdevices1.size()];
			for(int l=0;l<fogdevices1.size();l++)		
			{
				fogDevId[l]=fogdevices1.get(l).getId();
			}
			
			for(int i=0;i<populationsize;i++)
			{
				for(int j=0;j<tqueue2.size();j++)
				{
					String  module=(tqueue2.get(j).getKey()).getDestModuleName();
						//vmidArray[i][j]=tqueue2.get(j).getId();
					// initial_population[i][j]= findFogWithModule(fogdevices1, module).getId();
					initial_population[i][j]= fogDevId[r.nextInt(fogDevId.length-1)];
				}
			}
			return initial_population;
		 }
		/*
		 * EVALUATION PROCESS
		 */
		public double[] fitnessCalc(int[][] hostPop)
		{
		  //double util[]=new double[this.<PowerHost> getHostList().size()];
		  //double usedMips[]=new double[this.<PowerHost> getHostList().size()];
		  //double power[]=new double[this.<PowerHost> getHostList().size()];
		  double[] fitCal= new double[populationsize];
		
		  for(int k=0;k<fitCal.length;k++)
		  {
			 System.out.println();
			 System.out.println("chromozome number :"+k);
			 double util[]=new double[populationsize];
			 double usedMips[]=new double[100];
			 double power[]=new double[fogdevices1.size()];
			 for(int i=0;i<hostPop[k].length;i++)
			 {
				int id=hostPop[k][i];
				usedMips[id] +=Tqueue.get(i).getKey().getCloudletLength();
			 }
			 for(int i=0;i<util.length;i++)
			 {
				 if(findFogWithId(fogdevices1,i)!=null)
					 util[i]=(usedMips[i])/findFogWithId(fogdevices1,i).getHost().getTotalMips();
				 else
					 util[i]=0;
				 fitCal[i]=util[i];
				if(util[i]>1)
				{
					for(int j=0;j<fitCal.length;j++)
					{
						fitCal[j]=0;//this.fogdevices1.get(0).getHost().getMaxPower();
					}
					return fitCal;
				}
				System.out.print(i+":"+util[i]+"   ");
			 }
			 double present_power=0;
			 double max_power=0;
			/* for(int i=0;i<power.length;i++)
			 {
				if(findFogWithId(fogdevices1, i)!=null)
					power[i]=findFogWithId(fogdevices1, i).getHost().getPowerModel().getPower(util[i]);
				else
					power[i]=Double.MAX_VALUE;
				present_power+=power[i];
				if(findFogWithId(fogdevices1, i)!=null)
					max_power+=findFogWithId(fogdevices1, i).getHost().getPowerModel().getPower(1);
				else
					max_power+=0.1;
			 }
			 fitCal[k]=(present_power/max_power)*100;
			 System.out.println();
			 System.out.println("Power consumed :"+present_power);*/
		  }
		  return fitCal;
		}
		/*
		 * SELECTION PROCESS
		 */
		public int[][] selection(int[][] hostPop)
		{
			Random r=new Random();
			double[] fitness_cal=new double[hostPop.length];
			fitness_cal=fitnessCalc(hostPop);
			double[] fitness=new double[fitness_cal.length];
			double fit_total=0;
			for(int i=0;i<fitness.length;i++)
			{
				fitness[i]=1/(1+fitness_cal[i]);
				fit_total+=fitness[i];
			}
			double[] probability=new double[hostPop.length];
			for(int i=0;i<probability.length;i++)
			{
				probability[i]=fitness[i]/fit_total;
			}
			double cul_prob[]=new double[probability.length];
			for(int i=0;i<cul_prob.length;i++)
			{
				for(int j=0;j<=i;j++)
				{
					cul_prob[i]+=probability[j];
				}
			}
			double random_value[]=new double[probability.length];
			for(int i=0;i<random_value.length;i++)
			{
				random_value[i]=r.nextDouble();
			}
			int[][] newpopulation=new int[hostPop.length][this.Tqueue.size()];
			for(int i=0;i<random_value.length;i++)
			{
				for(int j=0;j<cul_prob.length;j++)
				{
					if(random_value[i]<cul_prob[j])
						continue;
					else
						newpopulation[i]=hostPop[j];
				}
			}
			hostPop=newpopulation;
			return hostPop;
		}
		/*
		 * CROSS OVER SELECTION
		 */
		public int[][] cross_selection(int[][] hostPop)
		{
			Random r1=new Random();
			List<Integer> cross_id=new ArrayList<Integer>();
			double[] r=new double[hostPop.length];
			int size=0;
			while(size<hostPop.length)
			{
				r[size]=r1.nextDouble();
				if(r[size]<cross_over_rate)
					cross_id.add(size);
				size++;
			}
			int[][] cross_pop=new int[cross_id.size()][this.Tqueue.size()];
			for(int i=0;i<cross_pop.length;i++)
			{
				cross_pop[i]=crossOver(hostPop[cross_id.get(i)],hostPop[cross_id.get((i+1)%cross_id.size())]);
			}
			for(int i=0;i<cross_id.size();i++)
			{
				hostPop[cross_id.get(i)]=cross_pop[i];
			}
			return hostPop;
		}
		/*
		 * CROSS OVER OPERATION
		 */
		public int[] crossOver(int[] list1, int[] list2)
		{
		/*	double[] used1=new double[this.<PowerHost> getHostList().size()];
			double[] used2=new double[this.<PowerHost> getHostList().size()];
			double[] utiled=new double[this.<PowerHost> getHostList().size()];
			int[] hosts=list1;
			for(int i=0;i<hosts.length;i++)
			{
				int id=hosts[i];
				used1[id] +=vmlist.get(i).getMips();
				int id2=list2[i];
				used2[id2] +=vmlist.get(i).getMips();
			}
			for(int i=0;i<used1.length;i++)
			{
				for(int k=i+1;k<used1.length;k++)
				{
					if(used1[i]<used1[k])
					{
						int temp2=hosts[i];
						hosts[i]=hosts[k];
						hosts[k]=temp2;
					}
					if(used2[i]<used2[k])
					{
						int temp3=list2[i];
						list2[i]=list2[k];
						list2[k]=temp3;
					}
				}			
			}
			Arrays.sort(used1);
			int ct=0;
			for(int s=0;s<used1.length;s++)
			{
				if(used1[s]<=50)
					ct++;
			}
			*/	
			int[] list=new int[this.Tqueue.size()];
			Random r=new Random();
			int max=this.Tqueue.size();
			int cutoff=r.nextInt(max-1);
			for(int i=0;i<cutoff;i++)
				list[i]=list1[i];
			for(int i=cutoff;i<list.length;i++)
				list[i]=list2[i];
			return list;
		/*	int cutoff=ct;
			for(int i=0,j=list2.length-1;i<cutoff;i++,j--)
				list1[i]=list2[j];*/
		}
		/*
		 * MUTATION OPERATION
		 */
			public int[][] mutate(int[][] hostPop)
			{
				/*double hostavailmips[]=new double[this.<PowerHost> getHostList().size()];
				double hostutil[]=new double[hostId.length];
				for(int j=0;j<this.<PowerHost> getHostList().size();j++)
				{
					hostavailmips[j]=this.<PowerHost> getHostList().get(j).getAvailableMips();
				}
				for(int i=0;i<hostId.length;i++)
				{
					hostutil[i]=(hostavailmips[hostId[i]]-vmlist.get(i).getMips())/this.<PowerHost> getHostList().get(hostId[i]).getTotalMips();
				}
				double increase[]=new double[hostId.length];
				for(int i=0;i<hostId.length;i++)
				{
					for(int j=i;j>=0;j--)
					{
						if(hostId[j]==hostId[i])
						{
							increase[j]=increase[i]-(hostutil[hostId[i]]-hostutil[hostId[j]]);
						//	hostutil[hostId[i]]+=util;
						}
					}
				//	System.out.println("====="+increase[i]+"====");
				}
				int minid=0;
				double min=increase[0];
				for(int i=0;i<increase.length;i++)
				{
					if(increase[i]<min)
					{
						min=increase[i];
						minid=hostId[i];
					}
				}
				int minid1=0;
				double nextmin=increase[0];
				for(int i=0;i<increase.length;i++)
				{
					if(increase[i]<nextmin && increase[i]>min)
					{
						min=increase[i];
						minid1=hostId[i];
					}
				}
				int id1=0;
				double max=increase[0];
				for(int i=0;i<increase.length;i++)
				{
					if(increase[i]>max)
					{
						max=increase[i];
						id1=hostId[i];
					}
				}
				int id2=0;
				double nmax=increase[0];
				for(int i=0;i<increase.length;i++)
				{
					if(increase[i]>nmax && increase[i]<max)
					{
						nmax=increase[i];
						id2=hostId[i];
					}
				}*/
				Random r=new Random();
				int total_length=hostPop.length*hostPop[0].length;
				int noofmutations=(int)(mutation_rate*total_length);
				for(int i=0;i<noofmutations;i++)
				{
					int number=(int) (1+Math.random()*(total_length-1));
					//hostPop[(number-1)/vmlist.size()][(number-1)%vmlist.size()]=r.nextInt(RandomConstants.NUMBER_OF_HOSTS-1);
				}
				return hostPop;
			}
			
		
		private FogDevice findFogWithId(List<FogDevice> fogdevices2, int id)
		{
			FogDevice fg=null;
			for(FogDevice f: fogdevices2)
			{
				if(f.getId()==id)
				{
					fg=f;
					return fg;
				}
			}	
			
			return null;
		}
			
	private FogDevice findFogWithModule(List<FogDevice> fogdevices2, String module)
	{
		FogDevice findfg=null;
		boolean flag=false;
		int s=fogdevices2.size();
		//System.out.print(s);
		int Check_array[];
		Check_array=new int[s];
		while(!flag)
		{
		Random r=new Random();
		int fg=r.nextInt(fogdevices2.size());		
		if(Check_array[fg]==1)
		{
			continue;
		}
		else
		{
			Check_array[fg]=1;
		}
		//Check_array[fg]=1;
		
		FogDevice fog1=fogdevices2.get(fg);
		appToModulesMap=((FogDevice) fog1).getAppToModulesMap();
		for (Map.Entry<String, List<String>> me :appToModulesMap.entrySet()) 
		{
			List<String> valueList = me.getValue();
			    for (String s1 : valueList) 
			  {
			   	  if(s1.equals(module))
			   	  {
			   		 findfg=fog1;
			   		  flag=true;
			   		//System.out.print(" "+fog1.getName());
			   		  break;
			  	  }			   	 
			  }
			    if(flag)
					break;				
		}
		if(flag)
			break;
		
		} 
		for(int j=0;j<s;j++)
		{
			Check_array[j]=0;
		}
		//System.out.println("Fog Device found is:"+ findfg);
			
			//fogdevices2.clear();
			return findfg;
		}



	private void processTupleFinished()
	{
		// TODO Auto-generated method stub
		
	}



	@Override
	public void shutdownEntity()
	{
		// TODO Auto-generated method stub
		
	}
	
	

}
