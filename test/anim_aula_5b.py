import plotly.express as px

fig = px.line(width=500, height=500)

r1 = [1]
r2 = [1]
t = []
K = 1
A = 0.5
n = 0.5
wtol = 0.1
dt=0.005
tmax=5



def fun(w):
    if abs(w)<= wtol:
        return -A*(w/wtol)
    else:
        return -A*(w/abs(w))

for i in range(round(tmax/dt)):
    t.append(i*dt)
    r1.append(r1[-1] -K*r1[-1]*dt)
    r2.append(r2[-1] +fun(r2[-1])*dt )

fig.add_scatter(x=t, y=r1, mode="lines", name="r1", line_color="red")
fig.add_scatter(x=t, y=r2, mode="lines", name="r2", line_color="limegreen")
fig.update_xaxes(title_text="t")
fig.update_yaxes(title_text="r")
fig.update_xaxes(showgrid=True, gridwidth=0.3, gridcolor="rgba(25,25,25,1)")
fig.update_yaxes(showgrid=True, gridwidth=0.3, gridcolor="rgba(25,25,25,1)")


fig.update_layout({'plot_bgcolor': 'rgba(25,25,25,1)','paper_bgcolor': 'rgba(25,25,25,1)', 'font_color' : 'white'})
fig.show()


fig = px.line(width=500, height=500)

f1 = []
f2 = []
r = []
for i in range(1000):
    r.append(-1 + 2*i/1000)
    f1.append(-K*r[-1])
    f2.append(fun(r[-1]))

fig.add_scatter(x=r, y=f1, mode="lines", name="f1", line_color="red")
fig.add_scatter(x=r, y=f2, mode="lines", name="f2", line_color="limegreen")
fig.update_xaxes(title_text="r")
fig.update_yaxes(title_text="f")
fig.update_xaxes(showgrid=True, gridwidth=0.3, gridcolor="rgba(25,25,25,1)")
fig.update_yaxes(showgrid=True, gridwidth=0.3, gridcolor="rgba(25,25,25,1)")

fig.update_layout({'plot_bgcolor': 'rgba(25,25,25,1)','paper_bgcolor': 'rgba(25,25,25,1)', 'font_color' : 'white'})
fig.show()