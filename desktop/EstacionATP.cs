// Lanzador de escritorio de la Estación ATP · Estudio.
// Levanta la estación y el trabajador de render en WSL, espera a que respondan, abre la app en su propia ventana (Edge en modo
// aplicación, sin barra del navegador) y, al cerrarla, ofrece detener todo. `EstacionATP.exe --detener` solo detiene.
using System;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Net;
using System.Reflection;
using System.Threading;
using System.Windows.Forms;

[assembly: AssemblyTitle("Estación ATP · Estudio")]
[assembly: AssemblyProduct("Estación ATP")]
[assembly: AssemblyCompany("Co.De Aerospace")]
[assembly: AssemblyVersion("1.0.0.0")]

static class Program
{
    const string Url = "http://127.0.0.1:8090";
    const string Distro = "Ubuntu-24.04";
    const string Repo = "~/estacion-atp";

    static int Wsl(string orden, int msMax)
    {
        var p = new Process();
        p.StartInfo.FileName = "wsl.exe";
        p.StartInfo.Arguments = "-d " + Distro + " -- bash -lc \"" + orden + "\"";
        p.StartInfo.UseShellExecute = false;
        p.StartInfo.CreateNoWindow = true;
        p.Start();
        if (!p.WaitForExit(msMax)) { try { p.Kill(); } catch { } return -1; }
        return p.ExitCode;
    }

    static bool Responde()
    {
        try
        {
            var r = (HttpWebRequest)WebRequest.Create(Url + "/api/health");
            r.Timeout = 2000;
            using (var resp = (HttpWebResponse)r.GetResponse()) return (int)resp.StatusCode == 200;
        }
        catch { return false; }
    }

    static string Navegador()
    {
        string[] rutas = {
            Environment.GetFolderPath(Environment.SpecialFolder.ProgramFilesX86) + @"\Microsoft\Edge\Application\msedge.exe",
            Environment.GetFolderPath(Environment.SpecialFolder.ProgramFiles) + @"\Microsoft\Edge\Application\msedge.exe",
            Environment.GetFolderPath(Environment.SpecialFolder.ProgramFiles) + @"\Google\Chrome\Application\chrome.exe",
            Environment.GetFolderPath(Environment.SpecialFolder.ProgramFilesX86) + @"\Google\Chrome\Application\chrome.exe" };
        foreach (var r in rutas) if (File.Exists(r)) return r;
        return null;
    }

    static void Detener()
    {
        Wsl("cd " + Repo + " && docker compose stop station; systemctl --user stop atp-render", 120000);
    }

    [STAThread]
    static int Main(string[] args)
    {
        if (args.Length > 0 && args[0] == "--detener") { Detener(); return 0; }

        bool nueva;
        using (var m = new Mutex(true, "EstacionATP-Lanzador", out nueva))
        {
            string nav = Navegador();
            if (!nueva)
            {   // ya hay una abierta: solo trae otra ventana de la app
                if (nav != null) Process.Start(nav, "--app=" + Url);
                return 0;
            }
            Application.EnableVisualStyles();

            var f = new Form
            {
                Text = "Estación ATP", FormBorderStyle = FormBorderStyle.FixedDialog, StartPosition = FormStartPosition.CenterScreen,
                ClientSize = new Size(440, 130), MaximizeBox = false, MinimizeBox = false,
                BackColor = Color.FromArgb(8, 15, 21), ForeColor = Color.FromArgb(242, 243, 244),
                Icon = Icon.ExtractAssociatedIcon(Application.ExecutablePath)
            };
            var titulo = new Label { Text = "Estación ATP · Estudio", Font = new Font("Segoe UI Semibold", 15f), AutoSize = true, Location = new Point(20, 16) };
            var estado = new Label { Text = "Preparando…", Font = new Font("Segoe UI", 10f), ForeColor = Color.FromArgb(0, 217, 255), AutoSize = true, Location = new Point(22, 60) };
            var barra = new ProgressBar { Style = ProgressBarStyle.Marquee, MarqueeAnimationSpeed = 30, Location = new Point(22, 92), Size = new Size(396, 8) };
            f.Controls.AddRange(new Control[] { titulo, estado, barra });

            Action<string> di = t => { try { f.BeginInvoke((Action)(() => estado.Text = t)); } catch { } };
            string error = null;
            Process app = null;

            var hilo = new Thread(() =>
            {
                try
                {
                    if (!Responde())
                    {
                        di("Arrancando la estación y el trabajador de render…");
                        int rc = Wsl("cd " + Repo + " && docker compose up -d station && (systemctl --user start atp-render || true)", 180000);
                        if (rc != 0) { error = "No se pudo arrancar la estación en WSL (código " + rc + "). ¿Está Docker corriendo en " + Distro + "?"; return; }
                        di("Esperando a que responda…");
                        for (int i = 0; i < 90 && !Responde(); i++) Thread.Sleep(2000);
                        if (!Responde()) { error = "La estación no respondió en " + Url + "."; return; }
                    }
                    di("Abriendo la app…");
                    if (nav == null) { Process.Start(Url); return; }
                    string perfil = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "EstacionATP", "perfil");
                    app = Process.Start(nav, "--app=" + Url + " --user-data-dir=\"" + perfil + "\" --window-size=1560,980 --no-first-run --no-default-browser-check");
                    Thread.Sleep(1500);
                }
                catch (Exception e) { error = e.Message; }
                finally { try { f.BeginInvoke((Action)(() => f.Close())); } catch { } }
            });
            f.Shown += (s, e) => hilo.Start();
            Application.Run(f);
            hilo.Join();

            if (error != null) { MessageBox.Show(error, "Estación ATP", MessageBoxButtons.OK, MessageBoxIcon.Error); return 1; }
            if (app == null) return 0;   // sin Edge ni Chrome: se abrió el navegador por defecto y la estación sigue en marcha
            app.WaitForExit();
            var r = MessageBox.Show("Cerraste la Estación ATP.\n\n¿Detener también la estación y el trabajador de render?\n(Si dices que no, siguen en segundo plano.)",
                "Estación ATP", MessageBoxButtons.YesNo, MessageBoxIcon.Question, MessageBoxDefaultButton.Button1);
            if (r == DialogResult.Yes) Detener();
            return 0;
        }
    }
}
