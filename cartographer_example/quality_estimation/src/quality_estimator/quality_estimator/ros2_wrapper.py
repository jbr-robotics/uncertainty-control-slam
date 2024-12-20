import subprocess


class Ros2Wrapper:
    def build_command(self):
        raise NotImplementedError("Subclasses must implement this method.")

    def run(self):
        # Run the constructed command
        command = self.build_command()
        try:
            print("Running command:", " ".join(command))
            process = subprocess.run(command, check=True, capture_output=True, text=True)
            print("Output:")
            print(process.stdout)
        except subprocess.CalledProcessError as e:
            print("Error executing command:", e)
            print("Error output:", e.stderr)