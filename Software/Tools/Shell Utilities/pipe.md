# Pipe (`|` in CLI)

The **pipe (`|`)** is a powerful feature in Unix-like shells that allows the output of one command to be used as the input of another. Pipes form the foundation of the "do one thing well" philosophy, enabling users to build complex workflows by chaining simple commands together. This is especially useful in robotics and engineering workflows where large amounts of data, logs, or process outputs need to be filtered, transformed, or analyzed quickly.

---

## ⚙️ Overview

- Pipes connect two or more commands together.
- Data flows **from left to right**: the output (stdout) of the command on the left becomes the input (stdin) of the command on the right.
- They enable modular command composition without temporary files.

---

## 🧠 Core Concepts

- **stdin (standard input)**: Input stream, typically from keyboard or another command.
- **stdout (standard output)**: Output stream, usually printed to the terminal.
- **stderr (standard error)**: Error messages, not passed through a pipe unless redirected.
- **Command Chaining**: Multiple pipes can be chained together for multi-step processing.

---

## 📊 Comparison Chart

| Concept           | Symbol | Usage Example                          | Notes                                      |
|-------------------|--------|-----------------------------------------|--------------------------------------------|
| Pipe              | `|`    | `ls -lt | head -n 5`                   | Connects stdout → stdin                     |
| Redirect Output   | `>`    | `ls > files.txt`                       | Saves stdout to a file                      |
| Append Output     | `>>`   | `echo "data" >> file.txt`              | Appends stdout to a file                    |
| Redirect Input    | `<`    | `sort < unsorted.txt`                  | Uses a file as stdin                        |
| Redirect Error    | `2>`   | `make 2> errors.log`                   | Captures stderr into a file                 |
| Redirect Both     | `&>`   | `command &> all_output.txt`            | Captures both stdout and stderr             |

---

## 🛠️ Use Cases

- Filtering logs for specific keywords
- Limiting or formatting command output
- Counting or sorting data without intermediate files
- Quick data inspection in robotics experiments

---

## ✅ Strengths

- Composable and flexible
- No need for intermediate files
- Built into all Unix-like systems
- Extremely lightweight

---

## ❌ Weaknesses

- Not suitable for complex data manipulation beyond line/text processing
- Performance overhead if overused in long pipelines
- Harder to debug than writing to files at each stage

---

## 🔧 Examples (One-Liners)

- Show the 5 most recent files in a directory:  
  `ls -lt | head -n 5`

- Count the number of running processes:  
  `ps aux | wc -l`

- Find lines containing "error" in logs:  
  `cat syslog.log | grep error`

- Display the top 10 largest files:  
  `du -ah | sort -rh | head -n 10`

- Show only IP addresses from network connections:  
  `netstat -tuln | awk '{print $5}' | cut -d: -f1 | sort | uniq`

- Count number of `.cpp` files in a project:  
  `find . -name "*.cpp" | wc -l`

- Show CPU usage of top 5 processes:  
  `ps aux --sort=-%cpu | head -n 6`

- List unique shell names in use:  
  `cat /etc/passwd | cut -d: -f7 | sort | uniq`

- Display only filenames from `ls -l`:  
  `ls -l | awk '{print $9}'`

- Find which commands are most used in history:  
  `history | awk '{print $2}' | sort | uniq -c | sort -nr | head`

---

- https://explainshell.com/explain?cmd=ls+-lt+%7C+head+-n+5
```
Pipelines
    A  pipeline is a sequence of one or more commands separated by one of the control operators | or |&.  The
    format for a pipeline is:

           [time [-p]] [ ! ] command [ [|⎪|&] command2 ... ]

    The standard output of command is connected  via  a  pipe  to  the  standard  input  of  command2.   This
    connection  is performed before any redirections specified by the command (see REDIRECTION below).  If |&
    is used, the standard error of command is connected to command2's standard input through the pipe; it  is
    shorthand  for  2>&1  |.   This  implicit  redirection  of  the  standard  error  is  performed after any
    redirections specified by the command.

    The return status of a pipeline is the exit status of the last command, unless  the  pipefail  option  is
    enabled.   If  pipefail  is  enabled,  the  pipeline's return status is the value of the last (rightmost)
    command to exit with a non-zero status, or zero if all commands exit successfully.  If the reserved  word
    !   precedes  a  pipeline, the exit status of that pipeline is the logical negation of the exit status as
    described above.  The shell waits for all commands in the pipeline to terminate before returning a value.

    If the time reserved word precedes a pipeline, the elapsed as well as user and system  time  consumed  by
    its execution are reported when the pipeline terminates.  The -p option changes the output format to that
    specified by POSIX.  When the shell is in posix mode, it does not recognize time as a  reserved  word  if
    the  next  token begins with a `-'.  The TIMEFORMAT variable may be set to a format string that specifies
    how the timing information should be displayed; see the description of TIMEFORMAT under  Shell  Variables
    below.

    When the shell is in posix mode, time may be followed by a newline.  In this case, the shell displays the
    total user and system time consumed by the shell and its children.  The TIMEFORMAT variable may  be  used
    to specify the format of the time information.

    Each command in a pipeline is executed as a separate process (i.e., in a subshell).
```

---

## 📚 Related Concepts

- [[Redirection]] (>, >>, <, 2>)
- [[Grep]] (pattern searching)
- [[AWK]] (pattern scanning and processing)
- [[Sed]] (stream editing)
- [[Unix Philosophy]]

---

## 🌐 External Resources

- [GNU Bash Manual – Pipelines](https://www.gnu.org/software/bash/manual/html_node/Pipelines.html)
- [Advanced Bash-Scripting Guide](https://tldp.org/LDP/abs/html/)
- [Unix Pipes Explained](https://explainshell.com/explain?cmd=ls+-lt+%7C+head+-n+5)
- https://explainshell.com/explain?cmd=ls+-lt+%7C+head+-n+5

---

## 🏆 Summary

Pipes (`|`) are one of the most powerful features of Unix-like shells, enabling the chaining of simple commands into more complex workflows. They remain essential for quick data inspection, automation, and engineering workflows where efficiency and flexibility matter. For robotics engineers, pipes provide a way to handle logs, simulation outputs, and sensor data efficiently without the need for custom scripts or file juggling.
