using System;
using System.Windows.Forms;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Collections.Generic;
using System.Diagnostics;

using Vx = delaunay.Triangulation.Vertex;


/// <summary>
/// Cтроит и отображает триангуляцию соотв. условиям Делоне в квадрате [-1.0, 1.0], используя простой итеративный алгоритм
/// </summary>
namespace delaunay
{
	/// <summary>
	/// Главное окно приложения. Выполняет функции прорисовки и обработки событий ввода.
	/// </summary>
	public class MainWnd : Form, IDrawable
	{

		static int Main(string[] args)
		{
			Trace.Listeners.Add(new ConsoleTraceListener());
			Trace.AutoFlush = true;

			Application.Run(new MainWnd(600, 600));
			return 0;
		}

		/// <summary>
		/// Initializes a new instance of the <see cref="delaunay.MainWnd"/> class.
		/// </summary>
		/// <param name="width">Width.</param>
		/// <param name="height">Height.</param>
		public MainWnd(int width, int height) 
		{
			_width = (float)width;
			_height = (float)height;

			ClientSize = new Size (width, height);
			Cursor = Cursors.Hand;

			Text = "Триангуляция Делоне";
		}

		protected override void OnPaint(PaintEventArgs e)
		{
			_graphics = this.CreateGraphics ();
			_graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;

			_graphics.Clear (Color.White);

			_width = (float)this.ClientSize.Width;
			_height = (float)this.ClientSize.Height;

			_triangulation.Draw (this);

			base.OnPaint (e);
		}

		/// <summary>
		/// Преобразует координаты из диапазона [-1.0, 1.0] в оконные 
		/// </summary>
		/// <returns>преобразованная точка</returns>
		/// <param name="x">абсцисса</param>
		/// <param name="y">ордината</param>
		private Point ConvertToWindowed(float x, float y)
		{
			float windowed_x = x * 0.5F + 0.5F, windowed_y = y * 0.5F + 0.5F;
			return new Point((int)(windowed_x * (_width - 1.0F)), (int)(windowed_y * (_height - 1.0F)));
		}

		/// <summary>
		/// реализует метод интерфейса для выполнения прорисовки треугольника и описывающей его окружности
		/// </summary>
		/// <param name="vertices">вершины треугольника</param>
		/// <param name="center">центр треугольника</param>
		/// <param name="radius">массив вершин</param>
		public void DrawTri(Vx [] vertices, Vx center, float radius)
		{
			var points = new Point[3];
			for (uint i = 0; i < 3; ++ i) {
				points [i] = ConvertToWindowed (vertices [i].x, vertices [i].y);
			}

			try {
				_graphics.DrawPolygon(new Pen(Color.Black), points);

				Point leftUpper = ConvertToWindowed(center.x - radius, center.y - radius);
				Point rightBottom = ConvertToWindowed(center.x + radius, center.y + radius);

				_graphics.DrawEllipse(new Pen(Color.FromArgb(64, Color.Red)), 
				                      new Rectangle(leftUpper.X, leftUpper.Y, rightBottom.X - leftUpper.X, rightBottom.Y - leftUpper.Y));

			} catch (Exception e) 
			{
				Trace.WriteLine ("Ошибка при отрисовке: " + e.Message);
			}
		}

		protected override void OnResize(EventArgs e)
		{
			Invalidate ();
			base.OnResize (e);
		}

		protected override void OnMouseClick (MouseEventArgs e)
		{
			var vx = new Vx { 
				x = (float)e.X / _width * 2.0F - 1.0F, 
				y = (float)e.Y / _height * 2.0F - 1.0F 
			};

			_triangulation.AddVertex (vx);

			Invalidate ();
			base.OnClick (e);
		}

		private float			_width, _height;
		private Triangulation	_triangulation = new Triangulation();
		private Graphics		_graphics;

	}

	/// <summary>
	/// Для построения триангуляции
	/// </summary>
	public class Triangulation
	{
		/// <summary>
		/// Массив всех вершин
		/// </summary>
		private List<Vertex> _points = new List<Vertex>();

		/// <summary>
		/// Список всех треугольников
		/// </summary>
		private LinkedList<Tri>	_tris = new LinkedList<Tri>();
		
	
		public struct Tri 
		{
			/// <summary>
			/// Индексы вершин образующих треугольник 
			/// </summary>
			public int[] indices;

			/// <summary>
			/// Центр описаной вокруг треугольника окружности
			/// </summary>
			public Vertex center;

			/// <summary>
			/// Квадрат радиуса описанной вокруг треугольника окружности
			/// </summary>
			public float radiusSq;
		}

		public struct Vertex 
		{
			public float x, y;
		}

		public struct Edge : IComparable
		{
			public int iFrom, iTo;

			public int CompareTo(object obj) {

				Edge edge = (Edge)obj;

				int ret = this.iTo - edge.iTo;

				if (ret == 0) {
					ret = this.iFrom - edge.iFrom;
				}

				return ret;
			}
						
		}


		/// <summary>
		/// Initializes a new instance of the <see cref="delaunay.Triangulation"/> class.
		/// </summary>
		public Triangulation()
		{
			_points.Add(new Vertex { x = -1.0F, y = -1.0F });
			_points.Add(new Vertex { x = 1.0F, y = -1.0F });
			_points.Add(new Vertex { x = 1.0F, y = 1.0F });
			_points.Add(new Vertex { x = -1.0F, y = 1.0F });
			_points.Add(new Vertex { x = -0.5F, y = -0.25F });

			AddTri (new [] { 4, 0, 1 });

			AddTri (new [] { 4, 1, 2 });
			AddTri (new [] { 4, 2, 3 });
			AddTri (new [] { 4, 3, 0 });
		}

		public void Draw(IDrawable drawable)
		{
			foreach (Tri tri in _tris)			
			{
				drawable.DrawTri(new [] { _points[tri.indices[0]], _points[tri.indices[1]], _points[tri.indices[2]] }, 
					tri.center, (float)Math.Sqrt(tri.radiusSq));
			}
		}

		class NearTris {
			/// <summary>
			/// Близкие треугольники - те которые не удовлетворяют условиям Делоне по отношению к новой вершине.
			/// Используются узлы списка треугольников всей триангуляции для быстрого удаления.
			/// </summary>
			public LinkedListNode<Tri>					nearest;
			public LinkedList< LinkedListNode<Tri> >	tris;
		};

		private float DotProduct(Vertex a, Vertex b)
		{
			return a.x * b.x + a.y * b.y;
		}

		private Vertex CalcVector(Vertex to, Vertex from)
		{
			return new Vertex { x = to.x - from.x, y = to.y - from.y };
		}

		private float CrossProduct(Vertex a, Vertex b)
		{
			return (a.x * b.y) - (a.y * b.x);
		}

		/// <summary>
		/// Determines whether this instance is embracing the specified vx tri.
		/// Определяет является ли входная точка внутренней точкой входного треугольника
		/// </summary>
		/// <returns><c>true</c> if this instance is embracing the specified vx tri; otherwise, <c>false</c>.</returns>
		/// <param name="vx">Вершина</param>
		/// <param name="tri">Треугольник</param>
		private bool IsEmbracing(Vertex vx, Tri tri)
		{
			bool isInner = true;

			for (uint i = 0; i < 3; ++ i) {

				Vertex edge = CalcVector (_points [tri.indices [(i + 1) % 3]], _points [tri.indices [i]]);
				Vertex toVx = CalcVector (vx, _points [tri.indices [i]]);

				if (CrossProduct (edge, toVx) < 0.0F)
					isInner = false;
			}

			return isInner;
		}

		/// <summary>
		/// Finds the near tris.
		/// Осуществляет поиск треугольников, нарушающих условие Делоне для новой входной точки
		/// </summary>
		/// <returns>The near tris.</returns>
		/// <param name="vx">Вершина</param>
		private NearTris FindNearTris(Vertex vx)
		{
			var nearTris = new NearTris { nearest = null, tris = new LinkedList< LinkedListNode<Tri> >() };

			for(LinkedListNode<Tri> node = _tris.First; node != null; node = node.Next) 
			{
				Tri tri = node.Value;

				Vertex diff = CalcVector (tri.center, vx);
				float distSq = diff.x * diff.x + diff.y * diff.y;

				if (distSq < tri.radiusSq) {

					nearTris.tris.AddLast (node);

					if (IsEmbracing (vx, tri)) {
						nearTris.nearest = node;
					}
				}
			}

			Trace.WriteLine ("near: " + nearTris.tris.Count); 

			return nearTris;
		}

		/// <summary>
		/// Фильтрует ребра не соответвующие обходу по часовой стрелке, т.е. только ребра контура
		/// </summary>
		/// <returns>Новый список ребер куда входят только ребра контура</returns>
		/// <param name="edges">Все ребра</param>
		/// <param name="wrongEdges">Ребра не соотв. обходу по часовой стрелке, т.е. ребра не принадлежащие контуру</param>
		private LinkedList<Edge> ClearFromWrongEdges(IEnumerable<Edge> edges, ICollection< Edge > wrongEdges)
		{
			var cleared = new LinkedList<Edge> ();

			foreach (Edge edge in edges) {
				if (!wrongEdges.Contains(edge)) cleared.AddLast (edge);
			}

			return cleared;
		}

		/// <summary>
		/// Классифицируем ребра, на основе этого заполняем списки. 
		/// </summary>
		/// <param name="iCurVx">Начальная точка ребра</param>
		/// <param name="iNextVx">Конечная точка ребра</param>
		/// <param name="edges">Правильные контурные ребра направленные (iCurVx, iNextVx) по часовой стрелке</param>
		/// <param name="wrongEdges">Ребра против часовой стрелки не могут быть контурными</param>
		private void ClassifyByDirection(Vx vx, int iCurVx, int iNextVx, ICollection<Edge> edges, 
		                                 ICollection< Edge > wrongEdges) {

			Vertex curEdge = CalcVector (_points [iCurVx], vx);
			Vertex nextEdge = CalcVector (_points [iNextVx], vx);

			float crossProduct = CrossProduct (nextEdge, curEdge);

			var edge = new Edge { iFrom = iCurVx, iTo = iNextVx };

			if (crossProduct < 0.0F) {
				edges.Add(edge);
			} else {
				//добавляем в обратном порядке для изменения направления на по часовой стрелке
				wrongEdges.Add (new Edge { iFrom = iNextVx, iTo = iCurVx });
				Trace.WriteLine ("wrong: " + edge.iFrom + " " + edge.iTo); 
			}
		}

		/// <summary>
		/// Формируем контур из совсем близких треугольников - треугольников нарушающих уловие Делоне
		/// </summary>
		/// <returns>Ребра контура</returns>
		/// <param name="nearTris">Близкие треугольники нарушающие условие Делоне</param>
		/// <param name="vx">Новая входная вершина</param>
		private LinkedList<Edge> FormEdgesFromNearTris(NearTris nearTris, Vertex vx)
		{
			var edges = new LinkedList<Edge>();
			var wrongEdges = new SortedSet<Edge>();

			foreach (LinkedListNode<Tri> node in nearTris.tris) {

				Tri tri = node.Value;

				for (uint i = 0; i < 3; ++ i) {

					int iCurVx = tri.indices [i], iNextVx = tri.indices [(i + 1) % 3];
					ClassifyByDirection (vx, iCurVx, iNextVx, edges, wrongEdges);
				}
			}	

			return ClearFromWrongEdges(edges, wrongEdges);
		}

		/// <summary>
		/// Добавляет указанную вершину в треангуляцию, производит ее перестроение для соотв. условию Делоне
		/// </summary>
		/// <param name="vx">Вершина</param>
		public void AddVertex(Vertex vx)
		{
			_points.Add (vx);
			int iNewVx = _points.Count - 1;

			NearTris nearTris = FindNearTris (vx);

			if (nearTris.tris.Count > 1) {

				LinkedList<Edge> edges = FormEdgesFromNearTris (nearTris, vx);

				foreach (LinkedListNode<Tri> triNode in nearTris.tris) {
					Trace.WriteLine ("remove tri: " + triNode.Value.indices[0] + " " + triNode.Value.indices[1] + " " + triNode.Value.indices[2]);
					_tris.Remove (triNode);
				}

				foreach(Edge edge in edges) {
					AddTri(new int[] { iNewVx, edge.iFrom, edge.iTo });
					Trace.WriteLine ("add tri: " + iNewVx + " " + edge.iFrom + " " + edge.iTo);
				}

				Trace.WriteLine ("overall tris: " + _tris.Count);
			} else
			{
				Tri tri = nearTris.nearest.Value;
				for (uint i = 0; i < 3; ++ i) {
					AddTri(new int[] { iNewVx, tri.indices[i], tri.indices[(i + 1) % 3] });
				}
				_tris.Remove (nearTris.nearest);
			}
		}

		/// <summary>
		/// Вычисляет атрибуты треугольника и заносит их в массивы.
		/// </summary>
		/// <param name="indices">Индексы вершин треугьлника в массиве вершин _vertices <see cref="delaunay.Triangulation._vertices"/></param>
		private void AddTri(int[] indices)
		{
			Vertex v1 = _points[indices[0]], v2 = _points[indices[1]], v3 = _points[indices[2]];

			float r1 = v1.x * v1.x + v1.y * v1.y;
			float r2 = v2.x * v2.x + v2.y * v2.y;
			float r3 = v3.x * v3.x + v3.y * v3.y;

			float a = v1.x * v2.y + v1.y * v3.x + v2.x * v3.y - v3.x * v2.y - v2.x * v1.y - v3.y * v1.x;
			float b = r1 * v2.y + v1.y * r3 + r2 * v3.y - r3 * v2.y - r2 * v1.y - v3.y * r1;
			float c = r1 * v2.x + v1.x * r3 + r2 * v3.x - r3 * v2.x - r2 * v1.x - v3.x * r1;

			var center = new Vertex { x = b / (a * 2.0F), y = -c / (a * 2.0F) };
			var delta = new Vertex { x = center.x - v1.x, y = center.y - v1.y };

			float radiusSq = delta.x * delta.x + delta.y * delta.y;

			_tris.AddLast (new Tri { indices = indices, center = center, radiusSq = radiusSq });
		}

	}

	/// <summary>
	/// Объект умеет рисовать
	/// </summary>
	public interface IDrawable 
	{
		/// <summary>
		/// выполнение прорисовки треугольника и описывающей его окружности
		/// </summary>
		/// <param name="vertices">вершины треугольника</param>
		/// <param name="center">центр треугольника</param>
		/// <param name="radius">массив вершин</param>
		void DrawTri (Vx [] vertices, Vx center, float radius);
	}


}

