import React from "react"
import Layout from "@theme-original/Layout"
import BookChatbot from "@site/src/components/HomepageFeatures/BookChatbot"
export default function LayoutWrapper(props: any) {
  return (
    <>
      <Layout {...props}>
        {props.children}
      </Layout>
      <BookChatbot /> {/* Floating chatbot on all pages */}
    </>
  )
}
