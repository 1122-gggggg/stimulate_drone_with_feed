import json, numpy as np
d=json.load(open('penghu.json'))['hourly']
t=np.array(d['time']); g=lambda k: np.array([np.nan if x is None else x for x in d[k]],float)
v10,gu,d10,v100,d100,T,P=[g(k) for k in ['wind_speed_10m','wind_gusts_10m','wind_direction_10m','wind_speed_100m','wind_direction_100m','temperature_2m','surface_pressure']]
mon=np.array([int(s[5:7]) for s in t]); w=np.isin(mon,[10,11,12,1,2,3])
print("n",len(t),"winter n",w.sum())
for name,m in [("winter",w),("all",np.ones_like(w))]:
    print(name,"v10 mean %.2f p50 %.2f p90 %.2f p99 %.2f max %.1f | v100 mean %.2f p90 %.2f p99 %.2f | gust mean %.2f p99 %.2f max %.1f"%(np.nanmean(v10[m]),*np.nanpercentile(v10[m],[50,90,99]),np.nanmax(v10[m]),np.nanmean(v100[m]),*np.nanpercentile(v100[m],[90,99]),np.nanmean(gu[m]),np.nanpercentile(gu[m],99),np.nanmax(gu[m])))
    print("  frac v10>=10.8: %.3f  >=13.9: %.3f ; frac gust>=13.9: %.3f"%(np.mean(v10[m]>=10.8),np.mean(v10[m]>=13.9),np.mean(gu[m]>=13.9)))
s=w&(v10>=8)
print("gust factor (v10>=8, winter): mean %.2f p90 %.2f"%(np.nanmean(gu[s]/v10[s]),np.nanpercentile(gu[s]/v10[s],90)))
print("v100/v10 ratio (v10>=8): %.3f"%np.nanmean(v100[s]/v10[s]))
dirs=d10[s]; b=np.histogram(dirs,bins=np.arange(-22.5,360,45))[0]
print("dir hist (N,NE,E,SE,S,SW,W,NW) strong winter:",b)
th=np.deg2rad(dirs); print("mean dir from %.0f deg"%(np.rad2deg(np.arctan2(np.nanmean(np.sin(th)),np.nanmean(np.cos(th))))%360))
rho=P*100/(287.05*(T+273.15)); print("rho winter mean %.3f"%np.nanmean(rho[w]))
i=np.nanargmax(gu); print("max gust at",t[i],gu[i],v10[i],d10[i])
