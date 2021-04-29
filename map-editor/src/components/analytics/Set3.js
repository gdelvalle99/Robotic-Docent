import React from "react";
import OutlinedCard from "./OutlinedCard";

export default function Set3() {
    const body = (question) => {
        return (
            <div>
                The most frequently asked question:
                <br />
                {question}
            </div>
        );
    };
    return (
        <div
            style={{
                display: "flex",
                justifyContent: "space-between",
                flexDirection: "column",
            }}
        >
            <div
                style={{
                    display: "flex",
                    justifyContent: "space-between",
                }}
            >
                <OutlinedCard
                    title="Maximum Number of Daily Interactions"
                    fact={56}
                    subfact="Interactions at Exhibit 1"
                    body={body("Who were the biggest contributors to this exhibit?")}
                />
                <OutlinedCard
                    title="Minimum Number of Daily Interactions"
                    fact={0}
                    subfact="Interactions at Exhibit 2"
                    body={body("Where can I find more about this artist's exhibit collection?")}
                />
            </div>
            <div
                style={{
                    display: "flex",
                    justifyContent: "space-between",
                }}
            >
                <OutlinedCard
                    title="Median Number of Daily Interactions"
                    fact={21}
                    subfact="Interactions Between All Exhibits"
                    body={body("Is there anyone that inspired these artists to create these pieces in this exhibit?")}
                />
                <OutlinedCard
                    title="Average Number of Daily Interactions"
                    fact={21.81}
                    subfact="Interactions Between All Exhibits"
                    body={body("In what time period were the pieces in this exhibit created?")}
                />
            </div>
        </div>
    );
}
