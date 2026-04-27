import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "Pi Bubble Mission Control",
  description: "Next.js operator console for Bubble soft gripper experiments on Raspberry Pi 5."
};

export default function RootLayout({
  children
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}
