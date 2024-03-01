## MPC

1. 估计/测量当前系统状态$y_k$

2. 基于$u_k$估计$y_{k+1}$也即$E_{k+1}$，基于$u_{k+1}$估计$y_{k+2}$也即$E_{k+2}$，依此类推直到$k+N$​进行最优化
   $$
   J=\sum_{k=1}^{N-1}E_k^TQE_k+u_k^TRu_k+E_N^TFE_N
   $$

3. 取优化后的$u_k$​(第一项)，依次滚动窗口计算

![image-20240229085453941](C:\Users\111\AppData\Roaming\Typora\typora-user-images\image-20240229085453941.png)



其中最优求解使用二次规划的形式，由于是对$u$求最优，所以
$$
J=\sum_{k=0}^{N-1}(E_k^TQE_k+u_k^TRu_k)+E_N^TFE_N
$$
中$E_N$必须得转换掉：通过系统状态方程(离散)
$$
X(k+1)=AX(k)+Bu(k)
$$
进行迭代，到最后把$E_N$代换为$u$相关和$E$的初值，即$X$的初值
$$
J = x(k)^TGx(k)+U(k)^THU(k)+2x(k)^TEU(k) \\
G=M^T\bar{Q}M, 
E=M^T\bar{Q}C, 
H=C^T\bar{Q}C+\bar{R}\\
$$
![image-20240229093607781](C:\Users\111\AppData\Roaming\Typora\typora-user-images\image-20240229093607781.png)

![image-20240229103257421](C:\Users\111\AppData\Roaming\Typora\typora-user-images\image-20240229103257421.png)

***注意***：以上是以$E_k=X_k-0$即镇定控制为基础的推导，如果是对reference$X_R$进行跟踪的跟踪控制，则需改为$E_k=X_k-X_r$，则$J$改为
$$
J = x(k)^TGx(k)+U(k)^THU(k)+2(x(k)^TM^T-X_R^T)EU(k) \\
E = \bar{Q}C
$$


#### 加入约束的MPC

使用方法和LQR一样，将约束不等式化为系数矩阵和常数向量

![image-20240301165732356](C:\Users\111\AppData\Roaming\Typora\typora-user-images\image-20240301165732356.png)

然后作为对应的实参放入quadprog函数

![image-20240301165755965](C:\Users\111\AppData\Roaming\Typora\typora-user-images\image-20240301165755965.png)



### 总结&使用

实际MPC的输入为当前最新的状态值，需要先设置好H矩阵,E矩阵，实际使用中$J$的最后一项系数不要用2。$Q$，$R$和$F$​为对角矩阵。







